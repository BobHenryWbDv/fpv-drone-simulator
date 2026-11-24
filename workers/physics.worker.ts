/**
 * Physics Worker - Runs at 1 kHz fixed timestep
 * Handles rigid-body dynamics, motor model, aerodynamics
 * Communicates via SharedArrayBuffer for zero-copy state sharing
 */

import { Vec3, Quat, Mat3, Random, GRAVITY, AIR_DENSITY, EPSILON } from '@core/math';

// ============================================================================
// Shared Memory Layout
// ============================================================================

// Physics State (written by physics worker, read by main thread & FC worker)
// All values in SI units, NED world frame, FLU body frame
const PHYSICS_STATE_STRIDE = 64; // floats per state snapshot
const PHYSICS_STATE_OFFSETS = {
  // Position (NED, meters)
  posX: 0, posY: 1, posZ: 2,
  // Velocity (NED, m/s)
  velX: 3, velY: 4, velZ: 5,
  // Orientation quaternion (world -> body, [x, y, z, w])
  quatX: 6, quatY: 7, quatZ: 8, quatW: 9,
  // Angular velocity (body frame, rad/s)
  angVelX: 10, angVelY: 11, angVelZ: 12,
  // Motor RPM (4 motors)
  motorRPM0: 13, motorRPM1: 14, motorRPM2: 15, motorRPM3: 16,
  // Motor thrust (N)
  motorThrust0: 17, motorThrust1: 18, motorThrust2: 19, motorThrust3: 20,
  // Motor torque (Nm)
  motorTorque0: 21, motorTorque1: 22, motorTorque2: 23, motorTorque3: 24,
  // Total force (body frame, N)
  forceX: 25, forceY: 26, forceZ: 27,
  // Total torque (body frame, Nm)
  torqueX: 28, torqueY: 29, torqueZ: 30,
  // Linear acceleration (body frame, m/s²)
  accelX: 31, accelY: 32, accelZ: 33,
  // Angular acceleration (body frame, rad/s²)
  angAccelX: 34, angAccelY: 35, angAccelZ: 36,
  // Wind velocity (world NED, m/s)
  windX: 37, windY: 38, windZ: 39,
  // Battery voltage (V)
  batteryVoltage: 40,
  // Battery current (A)
  batteryCurrent: 41,
  // Simulation time (seconds)
  simTime: 42,
  // Frame counter
  frameCount: 43,
  // Flags (bitmask)
  flags: 44,
  // Reserved for future expansion
  reservedStart: 45,
};

// Flag bits
const PHYSICS_FLAGS = {
  ARMED: 1 << 0,
  CRASHED: 1 << 1,
  ON_GROUND: 1 << 2,
  FAILSAFE: 1 << 3,
  GROUND_EFFECT: 1 << 4,
};

// Control Input (written by FC worker, read by physics worker)
const CONTROL_STRIDE = 16;
const CONTROL_OFFSETS = {
  // Motor commands (0-1 normalized)
  motorCmd0: 0, motorCmd1: 1, motorCmd2: 2, motorCmd3: 3,
  // Throttle (0-1)
  throttle: 4,
  // Target rates (body frame, rad/s) - for rate mode
  targetRateX: 5, targetRateY: 6, targetRateZ: 7,
  // Target angles (body frame, rad) - for angle mode
  targetAngleX: 8, targetAngleY: 9, targetAngleZ: 10,
  // Flight mode
  flightMode: 11,
  // Arming state
  armed: 12,
  // Timestamp (ms)
  timestamp: 13,
  // Sequence number
  sequence: 14,
  // Reserved
  reserved: 15,
};

// Flight modes
const FLIGHT_MODE = {
  ACRO: 0,
  ANGLE: 1,
  HORIZON: 2,
  AIR: 3,
  FAILSAFE: 4,
};

// ============================================================================
// Quadcopter Configuration
// ============================================================================

interface QuadConfig {
  // Mass properties
  mass: number;           // kg
  inertia: Vec3;          // Ixx, Iyy, Izz (kg·m²) - diagonal only
  // Geometry
  motorPositions: Vec3[]; // Body frame positions of 4 motors (FLU)
  motorDirections: number[]; // +1 CW, -1 CCW
  propDiameter: number;   // meters
  propPitch: number;      // meters
  // Motor model
  kThrust: number;        // N/(rad/s)²
  kDrag: number;          // Nm/(rad/s)²
  motorTimeConstant: number; // seconds
  maxRPM: number;         // RPM
  motorIdle: number;      // 0-1 normalized
  // Electrical
  batteryCells: number;   // e.g., 4 for 4S
  batteryCapacity: number; // Ah
  internalResistance: number; // Ohms
  voltageSagCoeff: number; // V/A
  // Aerodynamics
  dragCoeff: Vec3;        // Cx, Cy, Cz (body frame)
  dragArea: number;       // m²
  groundEffectFactor: number;
  // Camera mount
  cameraOffset: Vec3;     // Body frame position
  cameraAngle: number;    // Radians (pitch down positive)
}

// Default 5" quad configuration
const DEFAULT_QUAD_CONFIG: QuadConfig = {
  mass: 0.580, // kg (including battery)
  inertia: new Float32Array([0.0032, 0.0032, 0.0058]) as Vec3, // Ixx, Iyy, Izz
  motorPositions: [
    new Float32Array([ 0.115, -0.115, 0.015 ]) as Vec3, // Front-right (CW)
    new Float32Array([-0.115, -0.115, 0.015 ]) as Vec3, // Front-left (CCW)
    new Float32Array([-0.115,  0.115, 0.015 ]) as Vec3, // Rear-left (CW)
    new Float32Array([ 0.115,  0.115, 0.015 ]) as Vec3, // Rear-right (CCW)
  ],
  motorDirections: [1, -1, 1, -1], // CW, CCW, CW, CCW
  propDiameter: 0.127, // 5 inch
  propPitch: 0.08,     // 3.15 inch pitch
  kThrust: 1.15e-5,
  kDrag: 2.3e-7,
  motorTimeConstant: 0.018, // 18 ms
  maxRPM: 34000,
  motorIdle: 0.045,
  batteryCells: 4,
  batteryCapacity: 1.3, // 1300 mAh
  internalResistance: 0.045,
  voltageSagCoeff: 0.002,
  dragCoeff: new Float32Array([0.8, 0.8, 1.2]) as Vec3,
  dragArea: 0.015,
  groundEffectFactor: 0.8,
  cameraOffset: new Float32Array([0.025, 0, 0.015]) as Vec3,
  cameraAngle: 0.436, // 25 degrees
};

// ============================================================================
// Physics Worker State
// ============================================================================

interface PhysicsWorkerState {
  config: QuadConfig;
  // Shared buffers
  physicsState: Float32Array;
  controlInput: Float32Array;
  // Internal state
  motorOmega: Float32Array; // rad/s (internal motor speed)
  windWorld: Vec3;
  batteryVoltage: number;
  batteryCharge: number; // Ah remaining
  simTime: number;
  frameCount: number;
  lastControlSequence: number;
  // RNG for wind gusts
  rng: Random;
  // Sub-stepping
  motorSubSteps: number;
  // Performance tracking
  stepCount: number;
  lastPerfReport: number;
}

// ============================================================================
// Message Protocol
// ============================================================================

type WorkerMessage =
  | { type: 'init'; config: Partial<QuadConfig>; physicsBuffer: SharedArrayBuffer; controlBuffer: SharedArrayBuffer }
  | { type: 'step'; dt: number }
  | { type: 'reset'; position?: Vec3; velocity?: Vec3; orientation?: Quat }
  | { type: 'setConfig'; config: Partial<QuadConfig> }
  | { type: 'setWind'; wind: Vec3 }
  | { type: 'getState' }
  | { type: 'pause' }
  | { type: 'resume' };

type MainMessage =
  | { type: 'ready' }
  | { type: 'state'; physicsState: Float32Array }
  | { type: 'perf'; stepTime: number; avgStepTime: number }
  | { type: 'event'; event: string; data: any }
  | { type: 'error'; message: string };

// ============================================================================
// Physics Engine
// ============================================================================

class PhysicsEngine {
  private state: PhysicsWorkerState;
  private tempVec3_1 = new Float32Array(3) as Vec3;
  private tempVec3_2 = new Float32Array(3) as Vec3;
  private tempVec3_3 = new Float32Array(3) as Vec3;
  private tempQuat = new Float32Array(4) as Quat;
  private tempMat3 = new Float32Array(9) as Mat3;
  private gravity = new Float32Array([0, 0, GRAVITY]) as Vec3; // NED: positive down

  constructor() {
    this.state = this.createInitialState();
  }

  private createInitialState(): PhysicsWorkerState {
    return {
      config: { ...DEFAULT_QUAD_CONFIG },
      physicsState: new Float32Array(0),
      controlInput: new Float32Array(0),
      motorOmega: new Float32Array(4),
      windWorld: new Float32Array([0, 0, 0]) as Vec3,
      batteryVoltage: 16.8, // 4S full
      batteryCharge: 1.3,   // 1300 mAh
      simTime: 0,
      frameCount: 0,
      lastControlSequence: -1,
      rng: new Random(987654321),
      motorSubSteps: 0,
      stepCount: 0,
      lastPerfReport: performance.now(),
    };
  }

  initialize(message: WorkerMessage): void {
    const { config, physicsBuffer, controlBuffer } = message;
    
    this.state.physicsState = new Float32Array(physicsBuffer);
    this.state.controlInput = new Float32Array(controlBuffer);
    
    if (config) {
      this.state.config = { ...this.state.config, ...config };
    }
    
    // Initialize motor positions if not provided
    if (!config?.motorPositions) {
      this.state.config.motorPositions = DEFAULT_QUAD_CONFIG.motorPositions.map(p => new Float32Array(p) as Vec3);
    }
    
    // Reset simulation
    this.reset();
    
    // Calculate motor sub-steps
    const physicsDt = 0.001; // 1 ms
    this.state.motorSubSteps = Math.max(1, Math.round(this.state.config.motorTimeConstant / physicsDt / 5));
  }

  reset(position?: Vec3, velocity?: Vec3, orientation?: Quat): void {
    const s = this.state;
    const ps = s.physicsState;
    
    // Position
    if (position) {
      ps[PHYSICS_STATE_OFFSETS.posX] = position[0];
      ps[PHYSICS_STATE_OFFSETS.posY] = position[1];
      ps[PHYSICS_STATE_OFFSETS.posZ] = position[2];
    } else {
      ps[PHYSICS_STATE_OFFSETS.posX] = 0;
      ps[PHYSICS_STATE_OFFSETS.posY] = 0;
      ps[PHYSICS_STATE_OFFSETS.posZ] = 0;
    }
    
    // Velocity
    if (velocity) {
      ps[PHYSICS_STATE_OFFSETS.velX] = velocity[0];
      ps[PHYSICS_STATE_OFFSETS.velY] = velocity[1];
      ps[PHYSICS_STATE_OFFSETS.velZ] = velocity[2];
    } else {
      ps[PHYSICS_STATE_OFFSETS.velX] = 0;
      ps[PHYSICS_STATE_OFFSETS.velY] = 0;
      ps[PHYSICS_STATE_OFFSETS.velZ] = 0;
    }
    
    // Orientation
    if (orientation) {
      ps[PHYSICS_STATE_OFFSETS.quatX] = orientation[0];
      ps[PHYSICS_STATE_OFFSETS.quatY] = orientation[1];
      ps[PHYSICS_STATE_OFFSETS.quatZ] = orientation[2];
      ps[PHYSICS_STATE_OFFSETS.quatW] = orientation[3];
    } else {
      ps[PHYSICS_STATE_OFFSETS.quatX] = 0;
      ps[PHYSICS_STATE_OFFSETS.quatY] = 0;
      ps[PHYSICS_STATE_OFFSETS.quatZ] = 0;
      ps[PHYSICS_STATE_OFFSETS.quatW] = 1;
    }
    
    // Angular velocity
    ps[PHYSICS_STATE_OFFSETS.angVelX] = 0;
    ps[PHYSICS_STATE_OFFSETS.angVelY] = 0;
    ps[PHYSICS_STATE_OFFSETS.angVelZ] = 0;
    
    // Motors
    for (let i = 0; i < 4; i++) {
      ps[PHYSICS_STATE_OFFSETS.motorRPM0 + i] = 0;
      ps[PHYSICS_STATE_OFFSETS.motorThrust0 + i] = 0;
      ps[PHYSICS_STATE_OFFSETS.motorTorque0 + i] = 0;
      s.motorOmega[i] = 0;
    }
    
    // Forces/torques
    for (let i = 0; i < 3; i++) {
      ps[PHYSICS_STATE_OFFSETS.forceX + i] = 0;
      ps[PHYSICS_STATE_OFFSETS.torqueX + i] = 0;
      ps[PHYSICS_STATE_OFFSETS.accelX + i] = 0;
      ps[PHYSICS_STATE_OFFSETS.angAccelX + i] = 0;
    }
    
    // Wind
    ps[PHYSICS_STATE_OFFSETS.windX] = s.windWorld[0];
    ps[PHYSICS_STATE_OFFSETS.windY] = s.windWorld[1];
    ps[PHYSICS_STATE_OFFSETS.windZ] = s.windWorld[2];
    
    // Battery
    ps[PHYSICS_STATE_OFFSETS.batteryVoltage] = s.batteryVoltage;
    ps[PHYSICS_STATE_OFFSETS.batteryCurrent] = 0;
    
    // Time
    s.simTime = 0;
    s.frameCount = 0;
    ps[PHYSICS_STATE_OFFSETS.simTime] = 0;
    ps[PHYSICS_STATE_OFFSETS.frameCount] = 0;
    ps[PHYSICS_STATE_OFFSETS.flags] = PHYSICS_FLAGS.ON_GROUND;
    
    s.lastControlSequence = -1;
    s.stepCount = 0;
  }

  step(dt: number): void {
    const s = this.state;
    const ps = s.physicsState;
    const ci = s.controlInput;
    const config = s.config;
    
    const startTime = performance.now();
    
    // Read control input (with sequence check)
    const controlSeq = ci[CONTROL_OFFSETS.sequence];
    if (controlSeq !== s.lastControlSequence) {
      s.lastControlSequence = controlSeq;
    }
    
    const armed = ci[CONTROL_OFFSETS.armed] > 0.5;
    const flightMode = ci[CONTROL_OFFSETS.flightMode];
    const throttle = ci[CONTROL_OFFSETS.throttle];
    
    // Motor commands (0-1 normalized)
    const motorCmd = [
      ci[CONTROL_OFFSETS.motorCmd0],
      ci[CONTROL_OFFSETS.motorCmd1],
      ci[CONTROL_OFFSETS.motorCmd2],
      ci[CONTROL_OFFSETS.motorCmd3],
    ];
    
    // Update flags
    let flags = 0;
    if (armed) flags |= PHYSICS_FLAGS.ARMED;
    if (ps[PHYSICS_STATE_OFFSETS.posZ] <= 0.02 && Math.abs(ps[PHYSICS_STATE_OFFSETS.velZ]) < 0.1) {
      flags |= PHYSICS_FLAGS.ON_GROUND;
    }
    
    // Battery voltage sag
    this.updateBattery(motorCmd, dt);
    
    // Motor model sub-stepping
    this.updateMotors(motorCmd, throttle, armed, dt);
    
    // Aerodynamics
    this.updateAerodynamics(dt);
    
    // Rigid body integration (RK4)
    this.integrateRK4(dt);
    
    // Ground collision
    this.handleGroundCollision();
    
    // Update shared state
    ps[PHYSICS_STATE_OFFSETS.simTime] = s.simTime;
    ps[PHYSICS_STATE_OFFSETS.frameCount] = s.frameCount;
    ps[PHYSICS_STATE_OFFSETS.flags] = flags;
    ps[PHYSICS_STATE_OFFSETS.batteryVoltage] = s.batteryVoltage;
    ps[PHYSICS_STATE_OFFSETS.windX] = s.windWorld[0];
    ps[PHYSICS_STATE_OFFSETS.windY] = s.windWorld[1];
    ps[PHYSICS_STATE_OFFSETS.windZ] = s.windWorld[2];
    
    s.simTime += dt;
    s.frameCount++;
    s.stepCount++;
    
    // Performance reporting
    const stepTime = performance.now() - startTime;
    if (s.stepCount % 1000 === 0) {
      const now = performance.now();
      const avgStepTime = (now - s.lastPerfReport) / 1000;
      s.lastPerfReport = now;
      
      // Send perf to main thread
      (self as any).postMessage({
        type: 'perf',
        stepTime,
        avgStepTime,
      } as MainMessage);
    }
  }

  private updateBattery(motorCmd: number[], dt: number): void {
    const s = this.state;
    const config = s.config;
    
    // Calculate total current draw
    let totalCurrent = 0;
    for (let i = 0; i < 4; i++) {
      // Simplified: current proportional to thrust^1.5
      const thrust = Math.max(0, motorCmd[i]) * config.kThrust * Math.pow(config.maxRPM * 2 * Math.PI / 60, 2);
      totalCurrent += Math.pow(thrust / 10, 1.5); // Rough approximation
    }
    
    // Add baseline current (FC, RX, VTX, etc.)
    totalCurrent += 0.5; // ~500mA baseline
    
    // Voltage sag
    const sag = totalCurrent * config.internalResistance + s.batteryCharge * config.voltageSagCoeff;
    s.batteryVoltage = Math.max(
      config.batteryCells * 3.0, // 3.0V per cell minimum
      config.batteryCells * 4.2 - sag
    );
    
    // Update charge
    s.batteryCharge = Math.max(0, s.batteryCharge - totalCurrent * dt / 3600);
  }

  private updateMotors(motorCmd: number[], throttle: number, armed: boolean, dt: number): void {
    const s = this.state;
    const config = s.config;
    const ps = s.physicsState;
    
    const maxOmega = config.maxRPM * 2 * Math.PI / 60;
    const idleOmega = config.motorIdle * maxOmega;
    const tau = config.motorTimeConstant;
    
    const subDt = dt / s.motorSubSteps;
    
    for (let sub = 0; sub < s.motorSubSteps; sub++) {
      for (let i = 0; i < 4; i++) {
        let targetOmega = 0;
        
        if (armed && throttle > 0.01) {
          // Mix throttle with motor command
          const cmd = Math.max(config.motorIdle, Math.min(1, throttle * motorCmd[i]));
          targetOmega = cmd * maxOmega;
        } else if (armed) {
          targetOmega = idleOmega;
        }
        
        // First-order motor response
        const dOmega = (targetOmega - s.motorOmega[i]) * subDt / tau;
        s.motorOmega[i] += dOmega;
        s.motorOmega[i] = Math.max(0, Math.min(maxOmega, s.motorOmega[i]));
        
        // Calculate thrust and torque
        const omegaSq = s.motorOmega[i] * s.motorOmega[i];
        const thrust = config.kThrust * omegaSq;
        const torque = config.kDrag * omegaSq * config.motorDirections[i];
        
        ps[PHYSICS_STATE_OFFSETS.motorRPM0 + i] = s.motorOmega[i] * 60 / (2 * Math.PI);
        ps[PHYSICS_STATE_OFFSETS.motorThrust0 + i] = thrust;
        ps[PHYSICS_STATE_OFFSETS.motorTorque0 + i] = torque;
      }
    }
  }

  private updateAerodynamics(dt: number): void {
    const s = this.state;
    const ps = s.physicsState;
    const config = s.config;
    
    // Current state
    const posX = ps[PHYSICS_STATE_OFFSETS.posX];
    const posY = ps[PHYSICS_STATE_OFFSETS.posY];
    const posZ = ps[PHYSICS_STATE_OFFSETS.posZ];
    const velX = ps[PHYSICS_STATE_OFFSETS.velX];
    const velY = ps[PHYSICS_STATE_OFFSETS.velY];
    const velZ = ps[PHYSICS_STATE_OFFSETS.velZ];
    const quatX = ps[PHYSICS_STATE_OFFSETS.quatX];
    const quatY = ps[PHYSICS_STATE_OFFSETS.quatY];
    const quatZ = ps[PHYSICS_STATE_OFFSETS.quatZ];
    const quatW = ps[PHYSICS_STATE_OFFSETS.quatW];
    
    // World velocity
    const vWorld = this.tempVec3_1;
    vWorld[0] = velX; vWorld[1] = velY; vWorld[2] = velZ;
    
    // Wind velocity (world)
    const windWorld = s.windWorld;
    
    // Relative velocity (world)
    const vRelWorld = this.tempVec3_2;
    Vec3.sub(vRelWorld, vWorld, windWorld);
    
    // Transform to body frame
    const q = this.tempQuat;
    q[0] = quatX; q[1] = quatY; q[2] = quatZ; q[3] = quatW;
    const vRelBody = this.tempVec3_3;
    Vec3.transformQuat(vRelBody, vRelWorld, q); // Actually need inverse quat
    // Fix: transform by conjugate (world -> body)
    const qConj = this.tempQuat;
    qConj[0] = -quatX; qConj[1] = -quatY; qConj[2] = -quatZ; qConj[3] = quatW;
    Vec3.transformQuat(vRelBody, vRelWorld, qConj);
    
    // Drag force (body frame)
    const dragCoeff = config.dragCoeff;
    const dragArea = config.dragArea;
    const rho = AIR_DENSITY;
    
    const dragBody = this.tempVec3_1;
    for (let i = 0; i < 3; i++) {
      const v = vRelBody[i];
      dragBody[i] = -0.5 * rho * dragArea * dragCoeff[i] * v * Math.abs(v);
    }
    
    // Ground effect
    let groundEffectForce = 0;
    if (posZ > 0 && posZ < config.propDiameter * 1.5) {
      const totalThrust = 
        ps[PHYSICS_STATE_OFFSETS.motorThrust0] +
        ps[PHYSICS_STATE_OFFSETS.motorThrust1] +
        ps[PHYSICS_STATE_OFFSETS.motorThrust2] +
        ps[PHYSICS_STATE_OFFSETS.motorThrust3];
      
      const factor = config.groundEffectFactor * (config.propDiameter / (4 * Math.PI * posZ * posZ));
      groundEffectForce = factor * totalThrust;
      ps[PHYSICS_STATE_OFFSETS.flags] |= PHYSICS_FLAGS.GROUND_EFFECT;
    } else {
      ps[PHYSICS_STATE_OFFSETS.flags] &= ~PHYSICS_FLAGS.GROUND_EFFECT;
    }
    
    // Total thrust (body Z, negative = up in FLU)
    const totalThrust = 
      ps[PHYSICS_STATE_OFFSETS.motorThrust0] +
      ps[PHYSICS_STATE_OFFSETS.motorThrust1] +
      ps[PHYSICS_STATE_OFFSETS.motorThrust2] +
      ps[PHYSICS_STATE_OFFSETS.motorThrust3];
    
    // Total force in body frame
    const forceBody = this.tempVec3_2;
    forceBody[0] = dragBody[0];
    forceBody[1] = dragBody[1];
    forceBody[2] = dragBody[2] - totalThrust - groundEffectForce; // Z up in FLU
    
    ps[PHYSICS_STATE_OFFSETS.forceX] = forceBody[0];
    ps[PHYSICS_STATE_OFFSETS.forceY] = forceBody[1];
    ps[PHYSICS_STATE_OFFSETS.forceZ] = forceBody[2];
    
    // Total torque in body frame
    const torqueBody = this.tempVec3_3;
    torqueBody[0] = 0;
    torqueBody[1] = 0;
    torqueBody[2] = 0;
    
    // Motor torques
    for (let i = 0; i < 4; i++) {
      torqueBody[2] += ps[PHYSICS_STATE_OFFSETS.motorTorque0 + i]; // Yaw torque
    }
    
    // Motor thrust moments (roll/pitch)
    for (let i = 0; i < 4; i++) {
      const motorPos = config.motorPositions[i];
      const thrust = ps[PHYSICS_STATE_OFFSETS.motorThrust0 + i];
      // τ = r × F
      torqueBody[0] += motorPos[1] * thrust; // Roll: y * Fz
      torqueBody[1] -= motorPos[0] * thrust; // Pitch: -x * Fz
    }
    
    // Aerodynamic damping torque (simplified)
    const angVel = this.tempVec3_1;
    angVel[0] = ps[PHYSICS_STATE_OFFSETS.angVelX];
    angVel[1] = ps[PHYSICS_STATE_OFFSETS.angVelY];
    angVel[2] = ps[PHYSICS_STATE_OFFSETS.angVelZ];
    
    const dampingFactor = 0.001;
    torqueBody[0] -= angVel[0] * dampingFactor;
    torqueBody[1] -= angVel[1] * dampingFactor;
    torqueBody[2] -= angVel[2] * dampingFactor;
    
    ps[PHYSICS_STATE_OFFSETS.torqueX] = torqueBody[0];
    ps[PHYSICS_STATE_OFFSETS.torqueY] = torqueBody[1];
    ps[PHYSICS_STATE_OFFSETS.torqueZ] = torqueBody[2];
  }

  private integrateRK4(dt: number): void {
    const s = this.state;
    const ps = s.physicsState;
    const config = s.config;
    const mass = config.mass;
    const invMass = 1 / mass;
    const invInertia = this.tempVec3_1;
    invInertia[0] = 1 / config.inertia[0];
    invInertia[1] = 1 / config.inertia[1];
    invInertia[2] = 1 / config.inertia[2];
    
    // State vector: [pos(3), vel(3), quat(4), angVel(3)] = 13 elements
    const state = new Float32Array(13);
    const k1 = new Float32Array(13);
    const k2 = new Float32Array(13);
    const k3 = new Float32Array(13);
    const k4 = new Float32Array(13);
    const tempState = new Float32Array(13);
    
    // Pack current state
    state[0] = ps[PHYSICS_STATE_OFFSETS.posX];
    state[1] = ps[PHYSICS_STATE_OFFSETS.posY];
    state[2] = ps[PHYSICS_STATE_OFFSETS.posZ];
    state[3] = ps[PHYSICS_STATE_OFFSETS.velX];
    state[4] = ps[PHYSICS_STATE_OFFSETS.velY];
    state[5] = ps[PHYSICS_STATE_OFFSETS.velZ];
    state[6] = ps[PHYSICS_STATE_OFFSETS.quatX];
    state[7] = ps[PHYSICS_STATE_OFFSETS.quatY];
    state[8] = ps[PHYSICS_STATE_OFFSETS.quatZ];
    state[9] = ps[PHYSICS_STATE_OFFSETS.quatW];
    state[10] = ps[PHYSICS_STATE_OFFSETS.angVelX];
    state[11] = ps[PHYSICS_STATE_OFFSETS.angVelY];
    state[12] = ps[PHYSICS_STATE_OFFSETS.angVelZ];
    
    // RK4 stages
    this.derivatives(state, k1);
    this.addScaled(tempState, state, k1, dt * 0.5);
    this.derivatives(tempState, k2);
    this.addScaled(tempState, state, k2, dt * 0.5);
    this.derivatives(tempState, k3);
    this.addScaled(tempState, state, k3, dt);
    this.derivatives(tempState, k4);
    
    // Combine
    for (let i = 0; i < 13; i++) {
      state[i] += (dt / 6) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
    }
    
    // Normalize quaternion
    const qLen = Math.hypot(state[6], state[7], state[8], state[9]);
    if (qLen > 0) {
      state[6] /= qLen;
      state[7] /= qLen;
      state[8] /= qLen;
      state[9] /= qLen;
    }
    
    // Unpack back to shared memory
    ps[PHYSICS_STATE_OFFSETS.posX] = state[0];
    ps[PHYSICS_STATE_OFFSETS.posY] = state[1];
    ps[PHYSICS_STATE_OFFSETS.posZ] = state[2];
    ps[PHYSICS_STATE_OFFSETS.velX] = state[3];
    ps[PHYSICS_STATE_OFFSETS.velY] = state[4];
    ps[PHYSICS_STATE_OFFSETS.velZ] = state[5];
    ps[PHYSICS_STATE_OFFSETS.quatX] = state[6];
    ps[PHYSICS_STATE_OFFSETS.quatY] = state[7];
    ps[PHYSICS_STATE_OFFSETS.quatZ] = state[8];
    ps[PHYSICS_STATE_OFFSETS.quatW] = state[9];
    ps[PHYSICS_STATE_OFFSETS.angVelX] = state[10];
    ps[PHYSICS_STATE_OFFSETS.angVelY] = state[11];
    ps[PHYSICS_STATE_OFFSETS.angVelZ] = state[12];
    
    // Compute accelerations for telemetry
    const forceBody = this.tempVec3_1;
    forceBody[0] = ps[PHYSICS_STATE_OFFSETS.forceX];
    forceBody[1] = ps[PHYSICS_STATE_OFFSETS.forceY];
    forceBody[2] = ps[PHYSICS_STATE_OFFSETS.forceZ];
    
    const accelBody = this.tempVec3_2;
    accelBody[0] = forceBody[0] * invMass;
    accelBody[1] = forceBody[1] * invMass;
    accelBody[2] = forceBody[2] * invMass;
    
    ps[PHYSICS_STATE_OFFSETS.accelX] = accelBody[0];
    ps[PHYSICS_STATE_OFFSETS.accelY] = accelBody[1];
    ps[PHYSICS_STATE_OFFSETS.accelZ] = accelBody[2];
    
    const torqueBody = this.tempVec3_1;
    torqueBody[0] = ps[PHYSICS_STATE_OFFSETS.torqueX];
    torqueBody[1] = ps[PHYSICS_STATE_OFFSETS.torqueY];
    torqueBody[2] = ps[PHYSICS_STATE_OFFSETS.torqueZ];
    
    const angAccelBody = this.tempVec3_2;
    angAccelBody[0] = torqueBody[0] * invInertia[0];
    angAccelBody[1] = torqueBody[1] * invInertia[1];
    angAccelBody[2] = torqueBody[2] * invInertia[2];
    
    ps[PHYSICS_STATE_OFFSETS.angAccelX] = angAccelBody[0];
    ps[PHYSICS_STATE_OFFSETS.angAccelY] = angAccelBody[1];
    ps[PHYSICS_STATE_OFFSETS.angAccelZ] = angAccelBody[2];
  }

  private derivatives(state: Float32Array, deriv: Float32Array): void {
    const s = this.state;
    const config = s.config;
    const mass = config.mass;
    const invMass = 1 / mass;
    const invInertia = this.tempVec3_1;
    invInertia[0] = 1 / config.inertia[0];
    invInertia[1] = 1 / config.inertia[1];
    invInertia[2] = 1 / config.inertia[2];
    
    // Unpack state
    const px = state[0], py = state[1], pz = state[2];
    const vx = state[3], vy = state[4], vz = state[5];
    const qx = state[6], qy = state[7], qz = state[8], qw = state[9];
    const wx = state[10], wy = state[11], wz = state[12];
    
    // Position derivative = velocity
    deriv[0] = vx;
    deriv[1] = vy;
    deriv[2] = vz;
    
    // Velocity derivative = acceleration
    // Transform body force to world
    const forceBody = this.tempVec3_1;
    forceBody[0] = s.physicsState[PHYSICS_STATE_OFFSETS.forceX];
    forceBody[1] = s.physicsState[PHYSICS_STATE_OFFSETS.forceY];
    forceBody[2] = s.physicsState[PHYSICS_STATE_OFFSETS.forceZ];
    
    const q = this.tempQuat;
    q[0] = qx; q[1] = qy; q[2] = qz; q[3] = qw;
    
    const forceWorld = this.tempVec3_2;
    Vec3.transformQuat(forceWorld, forceBody, q); // body -> world
    
    // Add gravity (NED: positive down)
    deriv[3] = forceWorld[0] * invMass;
    deriv[4] = forceWorld[1] * invMass;
    deriv[5] = forceWorld[2] * invMass + GRAVITY;
    
    // Quaternion derivative = 0.5 * q ⊗ [0, ω]
    deriv[6] = 0.5 * (-qx * wx - qy * wy - qz * wz + qw * wx);
    deriv[7] = 0.5 * ( qx * wx - qy * wz + qz * wy + qw * wy);
    deriv[8] = 0.5 * ( qx * wy + qy * wx - qz * wz + qw * wz);
    deriv[9] = 0.5 * (-qx * wz + qy * wx - qz * wy + qw * 0); // Fixed
    
    // Wait, let me recompute quaternion derivative properly:
    // q̇ = 0.5 * q ⊗ [0, ωx, ωy, ωz]
    // q = [qx, qy, qz, qw] (x,y,z,w)
    // ω = [wx, wy, wz]
    // q̇ = 0.5 * [ qw*wx + qy*wz - qz*wy,
    //              qw*wy + qz*wx - qx*wz,
    //              qw*wz + qx*wy - qy*wx,
    //             -qx*wx - qy*wy - qz*wz ]
    deriv[6] = 0.5 * ( qw * wx + qy * wz - qz * wy );
    deriv[7] = 0.5 * ( qw * wy + qz * wx - qx * wz );
    deriv[8] = 0.5 * ( qw * wz + qx * wy - qy * wx );
    deriv[9] = 0.5 * (-qx * wx - qy * wy - qz * wz );
    
    // Angular velocity derivative = angular acceleration
    const torqueBody = this.tempVec3_1;
    torqueBody[0] = s.physicsState[PHYSICS_STATE_OFFSETS.torqueX];
    torqueBody[1] = s.physicsState[PHYSICS_STATE_OFFSETS.torqueY];
    torqueBody[2] = s.physicsState[PHYSICS_STATE_OFFSETS.torqueZ];
    
    // ω̇ = I⁻¹ * (τ - ω × (I * ω))
    // For diagonal inertia: I*ω = [Ixx*wx, Iyy*wy, Izz*wz]
    const Iomega = this.tempVec3_2;
    Iomega[0] = config.inertia[0] * wx;
    Iomega[1] = config.inertia[1] * wy;
    Iomega[2] = config.inertia[2] * wz;
    
    const omegaCrossIomega = this.tempVec3_3;
    Vec3.cross(omegaCrossIomega, 
      new Float32Array([wx, wy, wz]) as Vec3, 
      Iomega
    );
    
    deriv[10] = (torqueBody[0] - omegaCrossIomega[0]) * invInertia[0];
    deriv[11] = (torqueBody[1] - omegaCrossIomega[1]) * invInertia[1];
    deriv[12] = (torqueBody[2] - omegaCrossIomega[2]) * invInertia[2];
  }

  private addScaled(out: Float32Array, a: Float32Array, b: Float32Array, scale: number): void {
    for (let i = 0; i < a.length; i++) {
      out[i] = a[i] + b[i] * scale;
    }
  }

  private handleGroundCollision(): void {
    const s = this.state;
    const ps = s.physicsState;
    const config = s.config;
    
    const posZ = ps[PHYSICS_STATE_OFFSETS.posZ];
    const velZ = ps[PHYSICS_STATE_OFFSETS.velZ];
    
    // Ground at z=0 (NED: positive down, so ground is at z=0)
    // Quad is at positive z (above ground)
    if (posZ <= config.propDiameter * 0.5) {
      // Penetration depth
      const penetration = config.propDiameter * 0.5 - posZ;
      
      if (penetration > 0) {
        // Push up (negative Z in NED)
        ps[PHYSICS_STATE_OFFSETS.posZ] = config.propDiameter * 0.5;
        
        // Damping
        if (velZ > 0) { // Moving down
          ps[PHYSICS_STATE_OFFSETS.velZ] = -velZ * 0.3; // Bounce with damping
        }
        
        // Friction for horizontal velocity
        const friction = 0.9;
        ps[PHYSICS_STATE_OFFSETS.velX] *= friction;
        ps[PHYSICS_STATE_OFFSETS.velY] *= friction;
        
        // Angular velocity damping on ground
        const angDamping = 0.95;
        ps[PHYSICS_STATE_OFFSETS.angVelX] *= angDamping;
        ps[PHYSICS_STATE_OFFSETS.angVelY] *= angDamping;
        ps[PHYSICS_STATE_OFFSETS.angVelZ] *= angDamping;
        
        ps[PHYSICS_STATE_OFFSETS.flags] |= PHYSICS_FLAGS.ON_GROUND;
        
        // Crash detection
        if (velZ > 3) { // Hard impact > 3 m/s
          ps[PHYSICS_STATE_OFFSETS.flags] |= PHYSICS_FLAGS.CRASHED;
          (self as any).postMessage({
            type: 'event',
            event: 'crash',
            data: { impactVelocity: velZ, position: [ps[PHYSICS_STATE_OFFSETS.posX], ps[PHYSICS_STATE_OFFSETS.posY], ps[PHYSICS_STATE_OFFSETS.posZ]] }
          } as MainMessage);
        }
      }
    }
  }

  setWind(wind: Vec3): void {
    this.state.windWorld[0] = wind[0];
    this.state.windWorld[1] = wind[1];
    this.state.windWorld[2] = wind[2];
  }

  setConfig(config: Partial<QuadConfig>): void {
    this.state.config = { ...this.state.config, ...config };
  }

  getState(): Float32Array {
    return this.state.physicsState;
  }
}

// ============================================================================
// Worker Entry Point
// ============================================================================

const engine = new PhysicsEngine();
let running = true;
let animationFrameId: number | null = null;
const FIXED_DT = 0.001; // 1 ms = 1 kHz
let accumulator = 0;
let lastTime = performance.now();

function stepSimulation(timestamp: number): void {
  if (!running) return;
  
  const dt = (timestamp - lastTime) / 1000; // seconds
  lastTime = timestamp;
  
  accumulator += dt;
  
  // Fixed timestep loop
  while (accumulator >= FIXED_DT) {
    engine.step(FIXED_DT);
    accumulator -= FIXED_DT;
  }
  
  // Send state to main thread at render rate (throttled)
  if (engine.state.frameCount % 2 === 0) { // ~500 Hz state updates
    (self as any).postMessage({
      type: 'state',
      physicsState: engine.getState(),
    } as MainMessage);
  }
  
  animationFrameId = requestAnimationFrame(stepSimulation);
}

(self as any).onmessage = (event: MessageEvent<WorkerMessage>) => {
  const message = event.data;
  
  switch (message.type) {
    case 'init':
      engine.initialize(message);
      (self as any).postMessage({ type: 'ready' } as MainMessage);
      lastTime = performance.now();
      animationFrameId = requestAnimationFrame(stepSimulation);
      break;
      
    case 'step':
      engine.step(message.dt);
      break;
      
    case 'reset':
      engine.reset(message.position, message.velocity, message.orientation);
      break;
      
    case 'setConfig':
      engine.setConfig(message.config);
      break;
      
    case 'setWind':
      engine.setWind(message.wind);
      break;
      
    case 'getState':
      (self as any).postMessage({
        type: 'state',
        physicsState: engine.getState(),
      } as MainMessage);
      break;
      
    case 'pause':
      running = false;
      if (animationFrameId) {
        cancelAnimationFrame(animationFrameId);
        animationFrameId = null;
      }
      break;
      
    case 'resume':
      if (!running) {
        running = true;
        lastTime = performance.now();
        animationFrameId = requestAnimationFrame(stepSimulation);
      }
      break;
      
    default:
      console.warn('[PhysicsWorker] Unknown message type:', (message as any).type);
  }
};

// Handle errors
(self as any).onerror = (error: ErrorEvent) => {
  (self as any).postMessage({
    type: 'error',
    message: error.message,
  } as MainMessage);
};

export {}; // Make this a module