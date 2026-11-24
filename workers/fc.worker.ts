/**
 * Flight Controller Worker - Runs at 2-4 kHz (configurable)
 * Implements Betaflight 4.5+ compatible PID control, sensor fusion, mixer
 * Communicates with physics worker via SharedArrayBuffer
 */

import { Vec3, Quat, Mat3, Random, GRAVITY, DEG2RAD, RAD2DEG } from '@core/math';

// ============================================================================
// Shared Memory Layout (matches physics worker)
// ============================================================================

const PHYSICS_STATE_OFFSETS = {
  posX: 0, posY: 1, posZ: 2,
  velX: 3, velY: 4, velZ: 5,
  quatX: 6, quatY: 7, quatZ: 8, quatW: 9,
  angVelX: 10, angVelY: 11, angVelZ: 12,
  motorRPM0: 13, motorRPM1: 14, motorRPM2: 15, motorRPM3: 16,
  motorThrust0: 17, motorThrust1: 18, motorThrust2: 19, motorThrust3: 20,
  motorTorque0: 21, motorTorque1: 22, motorTorque2: 23, motorTorque3: 24,
  forceX: 25, forceY: 26, forceZ: 27,
  torqueX: 28, torqueY: 29, torqueZ: 30,
  accelX: 31, accelY: 32, accelZ: 33,
  angAccelX: 34, angAccelY: 35, angAccelZ: 36,
  windX: 37, windY: 38, windZ: 39,
  batteryVoltage: 40,
  batteryCurrent: 41,
  simTime: 42,
  frameCount: 43,
  flags: 44,
};

const CONTROL_OFFSETS = {
  motorCmd0: 0, motorCmd1: 1, motorCmd2: 2, motorCmd3: 3,
  throttle: 4,
  targetRateX: 5, targetRateY: 6, targetRateZ: 7,
  targetAngleX: 8, targetAngleY: 9, targetAngleZ: 10,
  flightMode: 11,
  armed: 12,
  timestamp: 13,
  sequence: 14,
  reserved: 15,
};

const FLIGHT_MODE = {
  ACRO: 0,
  ANGLE: 1,
  HORIZON: 2,
  AIR: 3,
  FAILSAFE: 4,
};

const PHYSICS_FLAGS = {
  ARMED: 1 << 0,
  CRASHED: 1 << 1,
  ON_GROUND: 1 << 2,
  FAILSAFE: 1 << 3,
  GROUND_EFFECT: 1 << 4,
};

// ============================================================================
// Configuration
// ============================================================================

interface PIDGains {
  P: number;
  I: number;
  D: number;
  F: number; // Feedforward
}

interface PIDProfile {
  pid: PIDGains[];        // [roll, pitch, yaw]
  dtermFilterType: number; // 0=off, 1=PT1, 2=PT2, 3=Biquad
  dtermFilterHz: number;   // Cutoff frequency
  itermWindup: number;     // I-term limit
  itermRelax: number;      // I-term relax factor
  itermRelaxType: number;  // 0=off, 1=gyro, 2=setpoint
  antiGravity: number;     // Anti-gravity gain
  antiGravityThreshold: number; // Throttle change threshold
  feedforwardTransition: number; // FF transition
  yawRateAccelLimit: number; // Yaw rate limit
  itermLimit: number;      // Max I-term
  dtermLimit: number;      // Max D-term
}

interface FCConfig {
  loopRate: number;        // Hz (2000 or 4000)
  gyroLpf: number;         // Gyro low-pass filter (1=PT1, 2=PT2, 3=Biquad)
  gyroLpfHz: number;       // Gyro LPF cutoff
  gyroSoftLpf: number;     // Software LPF
  gyroSoftLpfHz: number;
  dynLpf: number;          // Dynamic LPF (0=off, 1=on)
  dynLpfCurve: number;     // Curve exponent
  pidProfiles: PIDProfile[];
  activePidProfile: number;
  mixer: number;           // 0=QUADX, 1=QUADX_REVERSE, 2=QUADH, etc.
  motorPwmProtocol: number; // 0=DShot300, 1=DShot600, 2=DShot1200, 3=Multishot
  motorPoles: number;      // Motor poles for RPM filter
  rpmFilter: number;       // 0=off, 1=on
  rpmFilterHarmonics: number; // Number of harmonics
  dshotBidirectional: number; // 0=off, 1=on
  failsafeMode: number;    // 0=drop, 1=hover, 2=RTH
  failsafeDelay: number;   // Seconds before failsafe
  failsafeThrottle: number; // Throttle for hover failsafe
  smallAngle: number;      // Angle mode max angle (degrees)
  levelAngleLimit: number; // Level mode max angle (degrees)
  horizonTransition: number; // Horizon mode transition
  accelCutoff: number;     // Accelerometer LPF cutoff
  magEnabled: number;      // Magnetometer enabled
  baroEnabled: number;     // Barometer enabled
  gpsEnabled: number;      // GPS enabled
}

// Default Betaflight 4.5 configuration
const DEFAULT_FC_CONFIG: FCConfig = {
  loopRate: 2000,
  gyroLpf: 2,
  gyroLpfHz: 200,
  gyroSoftLpf: 1,
  gyroSoftLpfHz: 100,
  dynLpf: 1,
  dynLpfCurve: 3,
  pidProfiles: [
    // Profile 0: Race
    {
      pid: [
        { P: 55, I: 70, D: 28, F: 100 }, // Roll
        { P: 55, I: 70, D: 28, F: 100 }, // Pitch
        { P: 60, I: 45, D: 0, F: 100 },  // Yaw
      ],
      dtermFilterType: 1,
      dtermFilterHz: 30,
      itermWindup: 100,
      itermRelax: 50,
      itermRelaxType: 1,
      antiGravity: 350,
      antiGravityThreshold: 500,
      feedforwardTransition: 0,
      yawRateAccelLimit: 0,
      itermLimit: 400,
      dtermLimit: 0,
    },
    // Profile 1: Freestyle
    {
      pid: [
        { P: 50, I: 60, D: 30, F: 100 },
        { P: 50, I: 60, D: 30, F: 100 },
        { P: 55, I: 40, D: 0, F: 100 },
      ],
      dtermFilterType: 1,
      dtermFilterHz: 35,
      itermWindup: 100,
      itermRelax: 50,
      itermRelaxType: 1,
      antiGravity: 300,
      antiGravityThreshold: 500,
      feedforwardTransition: 0,
      yawRateAccelLimit: 0,
      itermLimit: 400,
      dtermLimit: 0,
    },
    // Profile 2: Cinematic
    {
      pid: [
        { P: 35, I: 40, D: 25, F: 80 },
        { P: 35, I: 40, D: 25, F: 80 },
        { P: 40, I: 30, D: 0, F: 80 },
      ],
      dtermFilterType: 2,
      dtermFilterHz: 50,
      itermWindup: 80,
      itermRelax: 30,
      itermRelaxType: 1,
      antiGravity: 200,
      antiGravityThreshold: 400,
      feedforwardTransition: 0,
      yawRateAccelLimit: 0,
      itermLimit: 300,
      dtermLimit: 0,
    },
    // Profile 3: Custom
    {
      pid: [
        { P: 45, I: 55, D: 28, F: 100 },
        { P: 45, I: 55, D: 28, F: 100 },
        { P: 50, I: 35, D: 0, F: 100 },
      ],
      dtermFilterType: 1,
      dtermFilterHz: 40,
      itermWindup: 100,
      itermRelax: 50,
      itermRelaxType: 1,
      antiGravity: 250,
      antiGravityThreshold: 450,
      feedforwardTransition: 0,
      yawRateAccelLimit: 0,
      itermLimit: 350,
      dtermLimit: 0,
    },
  ],
  activePidProfile: 0,
  mixer: 0, // QUADX
  motorPwmProtocol: 1, // DShot600
  motorPoles: 14,
  rpmFilter: 1,
  rpmFilterHarmonics: 3,
  dshotBidirectional: 1,
  failsafeMode: 0,
  failsafeDelay: 1.0,
  failsafeThrottle: 0.1,
  smallAngle: 45,
  levelAngleLimit: 55,
  horizonTransition: 50,
  accelCutoff: 15,
  magEnabled: 0,
  baroEnabled: 0,
  gpsEnabled: 0,
};

// ============================================================================
// Sensor Simulation
// ============================================================================

interface SensorData {
  gyro: Vec3;        // rad/s
  accel: Vec3;       // m/s²
  mag: Vec3;         // µT
  baro: number;      // Pa
  rpm: number[];     // 4 motors RPM
  timestamp: number;
}

class SensorSimulator {
  private config: FCConfig;
  private rng: Random;
  
  // Sensor biases
  private gyroBias = new Float32Array([0, 0, 0]) as Vec3;
  private accelBias = new Float32Array([0, 0, 0]) as Vec3;
  private magBias = new Float32Array([0, 0, 0]) as Vec3;
  
  // Noise parameters (ICM-42688 typical)
  private gyroNoiseDensity = 0.005; // rad/s/√Hz
  private accelNoiseDensity = 0.001; // m/s²/√Hz
  private magNoiseDensity = 0.1; // µT/√Hz
  
  // Temperature drift
  private tempCoeff = 0.0001;
  private simulatedTemp = 25; // °C
  
  // Vibration model
  private vibeFreq = 0;
  private vibeAmp = new Float32Array([0, 0, 0]) as Vec3;
  
  constructor(config: FCConfig) {
    this.config = config;
    this.rng = new Random(0xFCSENSOR);
    
    // Initialize biases
    this.gyroBias[0] = this.rng.nextNormal(0, 0.001);
    this.gyroBias[1] = this.rng.nextNormal(0, 0.001);
    this.gyroBias[2] = this.rng.nextNormal(0, 0.001);
    this.accelBias[0] = this.rng.nextNormal(0, 0.02);
    this.accelBias[1] = this.rng.nextNormal(0, 0.02);
    this.accelBias[2] = this.rng.nextNormal(0, 0.02);
  }
  
  updateConfig(config: Partial<FCConfig>): void {
    this.config = { ...this.config, ...config };
  }
  
  // Simulate gyroscope reading
  getGyro(trueAngVel: Vec3, motorRPMs: number[], dt: number): Vec3 {
    const out = new Float32Array(3) as Vec3;
    
    // True angular velocity
    out[0] = trueAngVel[0];
    out[1] = trueAngVel[1];
    out[2] = trueAngVel[2];
    
    // Add bias
    out[0] += this.gyroBias[0];
    out[1] += this.gyroBias[1];
    out[2] += this.gyroBias[2];
    
    // Add temperature drift (slow random walk)
    this.gyroBias[0] += this.rng.nextNormal(0, this.tempCoeff * dt);
    this.gyroBias[1] += this.rng.nextNormal(0, this.tempCoeff * dt);
    this.gyroBias[2] += this.rng.nextNormal(0, this.tempCoeff * dt);
    
    // Add vibration (motor harmonics)
    const avgRPM = (motorRPMs[0] + motorRPMs[1] + motorRPMs[2] + motorRPMs[3]) / 4;
    const freq = avgRPM / 60; // Hz
    const t = performance.now() / 1000;
    
    // Fundamental + harmonics
    for (let h = 1; h <= 4; h++) {
      const harmFreq = freq * h;
      const amp = 0.001 / h; // Decreasing amplitude
      out[0] += amp * Math.sin(2 * Math.PI * harmFreq * t + h);
      out[1] += amp * Math.sin(2 * Math.PI * harmFreq * t + h + 1);
      out[2] += amp * Math.sin(2 * Math.PI * harmFreq * t + h + 2);
    }
    
    // Add white noise
    const noiseStd = this.gyroNoiseDensity * Math.sqrt(this.config.loopRate);
    out[0] += this.rng.nextNormal(0, noiseStd);
    out[1] += this.rng.nextNormal(0, noiseStd);
    out[2] += this.rng.nextNormal(0, noiseStd);
    
    return out;
  }
  
  // Simulate accelerometer reading
  getAccel(trueAccel: Vec3, orientation: Quat, motorRPMs: number[]): Vec3 {
    const out = new Float32Array(3) as Vec3;
    const q = new Float32Array(4);
    q[0] = orientation[0]; q[1] = orientation[1]; q[2] = orientation[2]; q[3] = orientation[3];
    
    // True acceleration in world frame (NED)
    const worldAccel = new Float32Array(3);
    worldAccel[0] = trueAccel[0];
    worldAccel[1] = trueAccel[1];
    worldAccel[2] = trueAccel[2] + GRAVITY; // Add gravity (NED: down positive)
    
    // Transform to body frame
    const qConj = new Float32Array([-q[0], -q[1], -q[2], q[3]]);
    Vec3.transformQuat(out, worldAccel, qConj);
    
    // Add bias
    out[0] += this.accelBias[0];
    out[1] += this.accelBias[1];
    out[2] += this.accelBias[2];
    
    // Add motor vibration
    const avgRPM = (motorRPMs[0] + motorRPMs[1] + motorRPMs[2] + motorRPMs[3]) / 4;
    const freq = avgRPM / 60;
    const t = performance.now() / 1000;
    const vibeAmp = 0.05 * (avgRPM / 20000); // Scales with RPM
    
    out[0] += vibeAmp * Math.sin(2 * Math.PI * freq * t);
    out[1] += vibeAmp * Math.sin(2 * Math.PI * freq * t + 2);
    out[2] += vibeAmp * Math.sin(2 * Math.PI * freq * t + 4);
    
    // Add white noise
    const noiseStd = this.accelNoiseDensity * Math.sqrt(this.config.loopRate);
    out[0] += this.rng.nextNormal(0, noiseStd);
    out[1] += this.rng.nextNormal(0, noiseStd);
    out[2] += this.rng.nextNormal(0, noiseStd);
    
    return out;
  }
  
  // Simulate magnetometer
  getMag(orientation: Quat): Vec3 {
    const out = new Float32Array(3) as Vec3;
    
    // Earth magnetic field (approx 50 µT, inclination -60° in northern hemisphere)
    const earthMag = new Float32Array([25, 0, -43]) as Vec3; // µT in NED
    
    const q = new Float32Array(4);
    q[0] = orientation[0]; q[1] = orientation[1]; q[2] = orientation[2]; q[3] = orientation[3];
    const qConj = new Float32Array([-q[0], -q[1], -q[2], q[3]]);
    
    Vec3.transformQuat(out, earthMag, qConj);
    
    // Hard iron distortion
    out[0] += this.magBias[0];
    out[1] += this.magBias[1];
    out[2] += this.magBias[2];
    
    // Soft iron (simplified - just scaling)
    out[0] *= 1.05;
    out[1] *= 0.98;
    out[2] *= 1.02;
    
    // Noise
    const noiseStd = this.magNoiseDensity;
    out[0] += this.rng.nextNormal(0, noiseStd);
    out[1] += this.rng.nextNormal(0, noiseStd);
    out[2] += this.rng.nextNormal(0, noiseStd);
    
    return out;
  }
  
  // Simulate barometer
  getBaro(altitude: number): number {
    // Standard atmosphere model
    const P0 = 101325; // Pa at sea level
    const L = 0.0065;  // Temperature lapse rate K/m
    const T0 = 288.15; // Sea level temp K
    const g = 9.80665;
    const M = 0.0289644; // Molar mass kg/mol
    const R = 8.31447;   // Gas constant J/(mol·K)
    
    const pressure = P0 * Math.pow(1 - L * altitude / T0, g * M / (R * L));
    
    // Add noise
    return pressure + this.rng.nextNormal(0, 0.5); // ~0.5 Pa noise
  }
  
  // Get all sensors
  getAll(physicsState: Float32Array): SensorData {
    const trueAngVel = new Float32Array([
      physicsState[PHYSICS_STATE_OFFSETS.angVelX],
      physicsState[PHYSICS_STATE_OFFSETS.angVelY],
      physicsState[PHYSICS_STATE_OFFSETS.angVelZ],
    ]) as Vec3;
    
    const trueAccel = new Float32Array([
      physicsState[PHYSICS_STATE_OFFSETS.accelX],
      physicsState[PHYSICS_STATE_OFFSETS.accelY],
      physicsState[PHYSICS_STATE_OFFSETS.accelZ],
    ]) as Vec3;
    
    const orientation = new Float32Array([
      physicsState[PHYSICS_STATE_OFFSETS.quatX],
      physicsState[PHYSICS_STATE_OFFSETS.quatY],
      physicsState[PHYSICS_STATE_OFFSETS.quatZ],
      physicsState[PHYSICS_STATE_OFFSETS.quatW],
    ]) as Quat;
    
    const motorRPMs = [
      physicsState[PHYSICS_STATE_OFFSETS.motorRPM0],
      physicsState[PHYSICS_STATE_OFFSETS.motorRPM1],
      physicsState[PHYSICS_STATE_OFFSETS.motorRPM2],
      physicsState[PHYSICS_STATE_OFFSETS.motorRPM3],
    ];
    
    const altitude = -physicsState[PHYSICS_STATE_OFFSETS.posZ]; // NED: negative Z = up
    
    return {
      gyro: this.getGyro(trueAngVel, motorRPMs, 1/this.config.loopRate),
      accel: this.getAccel(trueAccel, orientation, motorRPMs),
      mag: this.getMag(orientation),
      baro: this.getBaro(altitude),
      rpm: motorRPMs,
      timestamp: physicsState[PHYSICS_STATE_OFFSETS.simTime] * 1000,
    };
  }
}

// ============================================================================
// Sensor Fusion (Complementary Filter - Betaflight style)
// ============================================================================

class SensorFusion {
  private q = new Float32Array([0, 0, 0, 1]) as Quat; // [x, y, z, w]
  private gyroLpf = new Float32Array(3); // Filtered gyro
  private accelLpf = new Float32Array(3); // Filtered accel
  private initialized = false;
  private alpha = 0.98; // Complementary filter coefficient
  
  // LPF state for gyro
  private gyroLpfState = new Float32Array(3);
  private accelLpfState = new Float32Array(3);
  
  reset(): void {
    this.q[0] = 0; this.q[1] = 0; this.q[2] = 0; this.q[3] = 1;
    this.gyroLpfState.fill(0);
    this.accelLpfState.fill(0);
    this.initialized = false;
  }
  
  // First-order low-pass filter
  private lpf1(out: Float32Array, state: Float32Array, input: Float32Array, alpha: number): void {
    for (let i = 0; i < 3; i++) {
      state[i] = state[i] + alpha * (input[i] - state[i]);
      out[i] = state[i];
    }
  }
  
  update(gyro: Vec3, accel: Vec3, dt: number, config: FCConfig): Quat {
    // Apply gyro LPF
    const gyroAlpha = dt * config.gyroLpfHz * 2 * Math.PI / (1 + dt * config.gyroLpfHz * 2 * Math.PI);
    this.lpf1(this.gyroLpf, this.gyroLpfState, gyro, gyroAlpha);
    
    // Apply accel LPF
    const accelAlpha = dt * config.accelCutoff * 2 * Math.PI / (1 + dt * config.accelCutoff * 2 * Math.PI);
    this.lpf1(this.accelLpf, this.accelLpfState, accel, accelAlpha);
    
    if (!this.initialized) {
      // Initialize from accelerometer
      const roll = Math.atan2(this.accelLpf[1], this.accelLpf[2]);
      const pitch = Math.atan2(-this.accelLpf[0], Math.hypot(this.accelLpf[1], this.accelLpf[2]));
      this.q = Quat.fromEuler(this.q, 0, pitch, roll);
      this.initialized = true;
      return this.q;
    }
    
    // Gyro integration
    const qDot = new Float32Array(4);
    const wx = this.gyroLpf[0], wy = this.gyroLpf[1], wz = this.gyroLpf[2];
    const qx = this.q[0], qy = this.q[1], qz = this.q[2], qw = this.q[3];
    
    qDot[0] = 0.5 * ( qw * wx + qy * wz - qz * wy );
    qDot[1] = 0.5 * ( qw * wy + qz * wx - qx * wz );
    qDot[2] = 0.5 * ( qw * wz + qx * wy - qy * wx );
    qDot[3] = 0.5 * (-qx * wx - qy * wy - qz * wz );
    
    this.q[0] += qDot[0] * dt;
    this.q[1] += qDot[1] * dt;
    this.q[2] += qDot[2] * dt;
    this.q[3] += qDot[3] * dt;
    
    // Normalize
    const len = Math.hypot(this.q[0], this.q[1], this.q[2], this.q[3]);
    this.q[0] /= len; this.q[1] /= len; this.q[2] /= len; this.q[3] /= len;
    
    // Complementary filter correction from accelerometer
    // Only correct if acceleration magnitude is close to 1g
    const accelMag = Math.hypot(this.accelLpf[0], this.accelLpf[1], this.accelLpf[2]);
    if (accelMag > 0.8 && accelMag < 1.2) {
      // Accelerometer-measured orientation
      const rollAcc = Math.atan2(this.accelLpf[1], this.accelLpf[2]);
      const pitchAcc = Math.atan2(-this.accelLpf[0], Math.hypot(this.accelLpf[1], this.accelLpf[2]));
      const qAcc = Quat.fromEuler(new Float32Array(4) as Quat, 0, pitchAcc, rollAcc);
      
      // Slerp towards accelerometer orientation
      Quat.slerp(this.q, this.q, qAcc, 1 - this.alpha);
    }
    
    return this.q;
  }
  
  getOrientation(): Quat {
    return this.q;
  }
}

// ============================================================================
// PID Controller (Betaflight 4.5+ compatible)
// ============================================================================

class PIDController {
  private config: FCConfig;
  private profile: PIDProfile;
  
  // State per axis (roll, pitch, yaw)
  private iTerm = new Float32Array(3);
  private prevError = new Float32Array(3);
  private prevDTerm = new Float32Array(3);
  private prevSetpoint = new Float32Array(3);
  private dtermFilterState = new Float32Array(3); // PT1 filter state
  private itermRelaxState = new Float32Array(3);
  private antiGravityState = new Float32Array(3);
  
  constructor(config: FCConfig) {
    this.config = config;
    this.profile = config.pidProfiles[config.activePidProfile];
  }
  
  updateConfig(config: Partial<FCConfig>): void {
    this.config = { ...this.config, ...config };
    this.profile = this.config.pidProfiles[this.config.activePidProfile];
  }
  
  setProfile(profileIndex: number): void {
    this.config.activePidProfile = profileIndex;
    this.profile = this.config.pidProfiles[profileIndex];
    this.reset();
  }
  
  reset(): void {
    this.iTerm.fill(0);
    this.prevError.fill(0);
    this.prevDTerm.fill(0);
    this.prevSetpoint.fill(0);
    this.dtermFilterState.fill(0);
    this.itermRelaxState.fill(0);
    this.antiGravityState.fill(0);
  }
  
  // PT1 low-pass filter for D-term
  private dtermLpf(input: number, state: number, alpha: number): number {
    return state + alpha * (input - state);
  }
  
  // Biquad filter (2nd order) for D-term
  private dtermBiquad(input: number, axis: number): number {
    // Simplified - would need proper biquad coefficients
    const alpha = 0.1;
    this.dtermFilterState[axis] = this.dtermFilterState[axis] + alpha * (input - this.dtermFilterState[axis]);
    return this.dtermFilterState[axis];
  }
  
  compute(
    axis: number,           // 0=roll, 1=pitch, 2=yaw
    setpoint: number,       // Target rate (rad/s) or angle (rad)
    measured: number,       // Measured rate (rad/s) or angle (rad)
    throttle: number,       // 0-1
    dt: number,
    isRateMode: boolean
  ): number {
    const pid = this.profile.pid[axis];
    const isYaw = axis === 2;
    
    // Setpoint weighting
    const setpointWeight = isYaw ? 0 : 0.5; // Betaflight default
    const error = setpointWeight * setpoint - measured;
    
    // P term
    const pTerm = pid.P * error;
    
    // I term
    const iGain = pid.I;
    this.iTerm[axis] += iGain * error * dt;
    
    // I-term limit
    const itermLimit = this.profile.itermLimit;
    this.iTerm[axis] = Math.max(-itermLimit, Math.min(itermLimit, this.iTerm[axis]));
    
    // I-term relax (reduce I-term near throttle limits)
    if (this.profile.itermRelax > 0) {
      const relaxFactor = 1 - this.profile.itermRelax / 100;
      if (throttle < 0.1 || throttle > 0.9) {
        this.iTerm[axis] *= relaxFactor;
      }
    }
    
    // Anti-gravity (boost I-term on rapid throttle changes)
    if (this.profile.antiGravity > 0 && !isYaw) {
      const throttleChange = Math.abs(throttle - this.prevSetpoint[axis]);
      if (throttleChange > this.profile.antiGravityThreshold / 1000) {
        this.antiGravityState[axis] = this.profile.antiGravity / 1000;
      }
      this.antiGravityState[axis] = Math.max(0, this.antiGravityState[axis] - dt * 10);
      this.iTerm[axis] += this.antiGravityState[axis] * error * dt;
    }
    
    // D term (derivative on measurement, not error)
    let dTerm = 0;
    if (pid.D > 0) {
      const dInput = -(measured - this.prevError[axis]) / dt; // Negative = derivative on measurement
      
      // D-term filter
      if (this.profile.dtermFilterType === 1) {
        // PT1
        const alpha = dt * this.profile.dtermFilterHz * 2 * Math.PI / (1 + dt * this.profile.dtermFilterHz * 2 * Math.PI);
        dTerm = this.dtermLpf(dInput, this.dtermFilterState[axis], alpha);
        this.dtermFilterState[axis] = dTerm;
      } else if (this.profile.dtermFilterType === 2) {
        // PT2 (two cascaded PT1)
        const alpha = dt * this.profile.dtermFilterHz * 2 * Math.PI / (1 + dt * this.profile.dtermFilterHz * 2 * Math.PI);
        const stage1 = this.dtermLpf(dInput, this.dtermFilterState[axis], alpha);
        dTerm = this.dtermLpf(stage1, this.dtermFilterState[axis], alpha);
      } else if (this.profile.dtermFilterType === 3) {
        // Biquad
        dTerm = this.dtermBiquad(dInput, axis);
      } else {
        // No filter
        dTerm = dInput;
      }
      
      dTerm *= pid.D;
      
      // D-term limit
      if (this.profile.dtermLimit > 0) {
        dTerm = Math.max(-this.profile.dtermLimit, Math.min(this.profile.dtermLimit, dTerm));
      }
    }
    
    // Feedforward
    let ffTerm = 0;
    if (pid.F > 0 && isRateMode) {
      const setpointRate = (setpoint - this.prevSetpoint[axis]) / dt;
      ffTerm = pid.F / 100 * setpointRate;
      
      // Feedforward transition
      if (this.profile.feedforwardTransition > 0) {
        const trans = this.profile.feedforwardTransition / 100;
        ffTerm *= trans;
      }
    }
    
    // Store for next iteration
    this.prevError[axis] = measured;
    this.prevSetpoint[axis] = setpoint;
    
    // Output
    let output = pTerm + this.iTerm[axis] + dTerm + ffTerm;
    
    // Yaw rate acceleration limit
    if (isYaw && this.profile.yawRateAccelLimit > 0) {
      const maxChange = this.profile.yawRateAccelLimit * dt;
      output = Math.max(output - maxChange, Math.min(output + maxChange, this.prevDTerm[axis]));
    }
    this.prevDTerm[axis] = output;
    
    return output;
  }
  
  getITerm(axis: number): number {
    return this.iTerm[axis];
  }
  
  getDTerm(axis: number): number {
    return this.prevDTerm[axis];
  }
}

// ============================================================================
// Mixer
// ============================================================================

class Mixer {
  private config: FCConfig;
  
  // Motor layout: [roll, pitch, yaw] per motor
  private layouts: number[][][] = [
    // QUADX
    [
      [1, -1, 1],   // Motor 0: FR (CW)
      [-1, -1, -1], // Motor 1: FL (CCW)
      [-1, 1, 1],   // Motor 2: RL (CW)
      [1, 1, -1],   // Motor 3: RR (CCW)
    ],
    // QUADX_REVERSE
    [
      [1, 1, -1],
      [-1, 1, 1],
      [-1, -1, -1],
      [1, -1, 1],
    ],
    // QUADH
    [
      [1, -1.5, 1],
      [-1, -1.5, -1],
      [-1, 1.5, 1],
      [1, 1.5, -1],
    ],
    // HEX6X
    [
      [1, -0.5, 1],
      [0.5, -1, -1],
      [-0.5, -1, 1],
      [-1, -0.5, -1],
      [-0.5, 1, -1],
      [0.5, 1, 1],
    ],
    // OCTOX8
    [
      [1, -0.5, 1],
      [0.5, -1, -1],
      [-0.5, -1, 1],
      [-1, -0.5, -1],
      [-1, 0.5, 1],
      [-0.5, 1, -1],
      [0.5, 1, 1],
      [1, 0.5, -1],
    ],
  ];
  
  constructor(config: FCConfig) {
    this.config = config;
  }
  
  updateConfig(config: Partial<FCConfig>): void {
    this.config = { ...this.config, ...config };
  }
  
  mix(throttle: number, rollOut: number, pitchOut: number, yawOut: number): number[] {
    const layout = this.layouts[this.config.mixer] || this.layouts[0];
    const motorCount = layout.length;
    const outputs = new Array(motorCount).fill(0);
    
    for (let i = 0; i < motorCount; i++) {
      outputs[i] = throttle +
        layout[i][0] * rollOut +
        layout[i][1] * pitchOut +
        layout[i][2] * yawOut;
    }
    
    return outputs;
  }
  
  getMotorCount(): number {
    return this.layouts[this.config.mixer]?.length || 4;
  }
}

// ============================================================================
// Flight Modes
// ============================================================================

class FlightModeManager {
  private config: FCConfig;
  private currentMode = FLIGHT_MODE.ACRO;
  private failsafeActive = false;
  private failsafeTimer = 0;
  private lastSignalTime = 0;
  
  constructor(config: FCConfig) {
    this.config = config;
  }
  
  updateConfig(config: Partial<FCConfig>): void {
    this.config = { ...this.config, ...config };
  }
  
  // Called with RC channel values (normalized -1 to 1)
  update(rcChannels: number[], dt: number): number {
    // Channel 5 (index 4) = flight mode switch
    const modeChannel = rcChannels[4] ?? 0;
    
    // Map channel to mode (3-position switch typical)
    let requestedMode = FLIGHT_MODE.ACRO;
    if (modeChannel < -0.5) requestedMode = FLIGHT_MODE.ANGLE;
    else if (modeChannel > 0.5) requestedMode = FLIGHT_MODE.HORIZON;
    
    // Arming channel (usually channel 5 or dedicated)
    const armChannel = rcChannels[4] ?? 0; // Simplified
    const armed = armChannel > 0;
    
    // Failsafe detection
    const signalPresent = Math.abs(rcChannels[0]) > 0.01 || Math.abs(rcChannels[1]) > 0.01;
    if (signalPresent) {
      this.lastSignalTime = 0;
      this.failsafeActive = false;
    } else {
      this.lastSignalTime += dt;
      if (this.lastSignalTime > this.config.failsafeDelay) {
        this.failsafeActive = true;
        requestedMode = FLIGHT_MODE.FAILSAFE;
      }
    }
    
    if (this.failsafeActive) {
      this.currentMode = FLIGHT_MODE.FAILSAFE;
    } else {
      this.currentMode = requestedMode;
    }
    
    return this.currentMode;
  }
  
  getMode(): number {
    return this.currentMode;
  }
  
  isFailsafe(): boolean {
    return this.failsafeActive;
  }
  
  getFailsafeThrottle(): number {
    return this.config.failsafeThrottle;
  }
}

// ============================================================================
// CLI Parser (Betaflight subset)
// ============================================================================

class CLIParser {
  private config: FCConfig;
  
  constructor(config: FCConfig) {
    this.config = config;
  }
  
  parse(command: string): { success: boolean; response: string } {
    const parts = command.trim().split(/\s+/);
    if (parts.length === 0) return { success: true, response: '' };
    
    const cmd = parts[0].toLowerCase();
    const args = parts.slice(1);
    
    switch (cmd) {
      case 'set':
        return this.handleSet(args);
      case 'get':
        return this.handleGet(args);
      case 'save':
        return { success: true, response: 'Config saved (simulated)' };
      case 'defaults':
        return { success: true, response: 'Defaults loaded (simulated)' };
      case 'pid':
        return this.handlePid(args);
      case 'profile':
        return this.handleProfile(args);
      case 'mixer':
        return this.handleMixer(args);
      case 'motor':
        return this.handleMotor(args);
      case 'failsafe':
        return this.handleFailsafe(args);
      case 'status':
        return this.handleStatus();
      case 'version':
        return { success: true, response: 'Betaflight 4.5.0 (FPV Sim Build)' };
      case 'help':
        return this.handleHelp();
      default:
        return { success: false, response: `Unknown command: ${cmd}` };
    }
  }
  
  private handleSet(args: string[]): { success: boolean; response: string } {
    if (args.length < 2) return { success: false, response: 'Usage: set <name> <value>' };
    
    const name = args[0];
    const value = args[1];
    const numValue = parseFloat(value);
    
    // Handle nested config paths (e.g., pid_profile_0_pid)
    if (name.startsWith('pid_profile_')) {
      const match = name.match(/pid_profile_(\d+)_(.+)/);
      if (match) {
        const profileIdx = parseInt(match[1]);
        const param = match[2];
        return this.setPidProfileParam(profileIdx, param, value);
      }
    }
    
    // Direct config properties
    if (name in this.config) {
      (this.config as any)[name] = isNaN(numValue) ? value : numValue;
      return { success: true, response: `${name} = ${value}` };
    }
    
    return { success: false, response: `Unknown setting: ${name}` };
  }
  
  private handleGet(args: string[]): { success: boolean; response: string } {
    if (args.length === 0) {
      // Dump all settings
      return { success: true, response: JSON.stringify(this.config, null, 2) };
    }
    
    const name = args[0];
    if (name in this.config) {
      return { success: true, response: `${name} = ${(this.config as any)[name]}` };
    }
    
    return { success: false, response: `Unknown setting: ${name}` };
  }
  
  private handlePid(args: string[]): { success: boolean; response: string } {
    if (args.length === 0) {
      const profile = this.config.pidProfiles[this.config.activePidProfile];
      let out = `PID Profile ${this.config.activePidProfile}:\n`;
      ['Roll', 'Pitch', 'Yaw'].forEach((axis, i) => {
        const pid = profile.pid[i];
        out += `${axis}: P=${pid.P} I=${pid.I} D=${pid.D} F=${pid.F}\n`;
      });
      out += `Dterm Filter: type=${profile.dtermFilterType} hz=${profile.dtermFilterHz}\n`;
      out += `Iterm: windup=${profile.itermWindup} relax=${profile.itermRelax} type=${profile.itermRelaxType}\n`;
      out += `Anti-Gravity: ${profile.antiGravity} (thresh=${profile.antiGravityThreshold})\n`;
      return { success: true, response: out };
    }
    
    return { success: false, response: 'Usage: pid' };
  }
  
  private handleProfile(args: string[]): { success: boolean; response: string } {
    if (args.length === 0) {
      return { success: true, response: `Active profile: ${this.config.activePidProfile}` };
    }
    
    const idx = parseInt(args[0]);
    if (idx >= 0 && idx < this.config.pidProfiles.length) {
      this.config.activePidProfile = idx;
      return { success: true, response: `Switched to PID profile ${idx}` };
    }
    
    return { success: false, response: 'Invalid profile index' };
  }
  
  private handleMixer(args: string[]): { success: boolean; response: string } {
    if (args.length === 0) {
      const mixers = ['QUADX', 'QUADX_REVERSE', 'QUADH', 'HEX6X', 'OCTOX8'];
      return { success: true, response: `Mixer: ${mixers[this.config.mixer] || 'UNKNOWN'}` };
    }
    
    const mixers = ['QUADX', 'QUADX_REVERSE', 'QUADH', 'HEX6X', 'OCTOX8'];
    const idx = mixers.indexOf(args[0].toUpperCase());
    if (idx >= 0) {
      this.config.mixer = idx;
      return { success: true, response: `Mixer set to ${mixers[idx]}` };
    }
    
    return { success: false, response: `Unknown mixer: ${args[0]}` };
  }
  
  private handleMotor(args: string[]): { success: boolean; response: string } {
    if (args.length === 0) {
      const protocols = ['DShot300', 'DShot600', 'DShot1200', 'Multishot', 'Oneshot125'];
      return { success: true, response: `Motor protocol: ${protocols[this.config.motorPwmProtocol]}` };
    }
    
    const protocols = ['DShot300', 'DShot600', 'DShot1200', 'Multishot', 'Oneshot125'];
    const idx = protocols.indexOf(args[0]);
    if (idx >= 0) {
      this.config.motorPwmProtocol = idx;
      return { success: true, response: `Motor protocol set to ${protocols[idx]}` };
    }
    
    return { success: false, response: 'Unknown protocol' };
  }
  
  private handleFailsafe(args: string[]): { success: boolean; response: string } {
    if (args.length === 0) {
      const modes = ['Drop', 'Hover', 'RTH'];
      return { success: true, response: `Failsafe: ${modes[this.config.failsafeMode]} delay=${this.config.failsafeDelay}s throttle=${this.config.failsafeThrottle}` };
    }
    
    return { success: false, response: 'Usage: failsafe' };
  }
  
  private handleStatus(): { success: boolean; response: string } {
    const profile = this.config.pidProfiles[this.config.activePidProfile];
    let out = `=== FC Status ===\n`;
    out += `Loop Rate: ${this.config.loopRate} Hz\n`;
    out += `PID Profile: ${this.config.activePidProfile}\n`;
    out += `Mixer: ${['QUADX','QUADX_REV','QUADH','HEX6X','OCTOX8'][this.config.mixer]}\n`;
    out += `Motor Protocol: ${['DShot300','DShot600','DShot1200','Multishot','Oneshot125'][this.config.motorPwmProtocol]}\n`;
    out += `RPM Filter: ${this.config.rpmFilter ? 'ON' : 'OFF'} (${this.config.rpmFilterHarmonics} harmonics)\n`;
    out += `Gyro LPF: ${this.config.gyroLpf} @ ${this.config.gyroLpfHz}Hz\n`;
    out += `Dyn LPF: ${this.config.dynLpf ? 'ON' : 'OFF'}\n`;
    out += `PID: R(P${profile.pid[0].P} I${profile.pid[0].I} D${profile.pid[0].D}) P(P${profile.pid[1].P} I${profile.pid[1].I} D${profile.pid[1].D}) Y(P${profile.pid[2].P} I${profile.pid[2].I} D${profile.pid[2].D})\n`;
    return { success: true, response: out };
  }
  
  private handleHelp(): { success: boolean; response: string } {
    return {
      success: true,
      response: `Available commands:
set <name> <value>     - Set configuration value
get [name]             - Get configuration value(s)
save                   - Save configuration
defaults               - Load defaults
pid                    - Show PID profile
profile <0-3>          - Switch PID profile
mixer <type>           - Set mixer (QUADX, QUADH, HEX6X, OCTOX8)
motor <protocol>       - Set motor protocol
failsafe               - Show failsafe config
status                 - Show FC status
version                - Show version
help                   - This help`
    };
  }
  
  private setPidProfileParam(profileIdx: number, param: string, value: string): { success: boolean; response: string } {
    if (profileIdx >= this.config.pidProfiles.length) {
      return { success: false, response: 'Invalid profile index' };
    }
    
    const profile = this.config.pidProfiles[profileIdx];
    const numValue = parseFloat(value);
    
    if (param === 'pid') {
      // Format: "P I D F" for roll pitch yaw
      const parts = value.split(/\s+/).map(parseFloat);
      if (parts.length === 12) {
        for (let i = 0; i < 3; i++) {
          profile.pid[i].P = parts[i * 4];
          profile.pid[i].I = parts[i * 4 + 1];
          profile.pid[i].D = parts[i * 4 + 2];
          profile.pid[i].F = parts[i * 4 + 3];
        }
        return { success: true, response: `PID profile ${profileIdx} updated` };
      }
      return { success: false, response: 'PID requires 12 values (P I D F for 3 axes)' };
    }
    
    if (param in profile && typeof (profile as any)[param] === 'number') {
      (profile as any)[param] = isNaN(numValue) ? value : numValue;
      return { success: true, response: `${param} = ${value}` };
    }
    
    return { success: false, response: `Unknown PID parameter: ${param}` };
  }
}

// ============================================================================
// Flight Controller Worker Main
// ============================================================================

interface FCWorkerState {
  config: FCConfig;
  physicsState: Float32Array;
  controlInput: Float32Array;
  
  sensorSim: SensorSimulator;
  sensorFusion: SensorFusion;
  pidController: PIDController;
  mixer: Mixer;
  flightModeManager: FlightModeManager;
  cliParser: CLIParser;
  
  // Input state
  rcChannels: number[]; // 16 channels, -1 to 1
  lastRcUpdate: number;
  
  // Timing
  loopDt: number;
  lastLoopTime: number;
  loopCount: number;
  
  // CLI
  cliBuffer: string;
  
  // Telemetry
  lastTelemSent: number;
}

const engine: FCWorkerState = {
  config: { ...DEFAULT_FC_CONFIG },
  physicsState: new Float32Array(0),
  controlInput: new Float32Array(0),
  sensorSim: new SensorSimulator(DEFAULT_FC_CONFIG),
  sensorFusion: new SensorFusion(),
  pidController: new PIDController(DEFAULT_FC_CONFIG),
  mixer: new Mixer(DEFAULT_FC_CONFIG),
  flightModeManager: new FlightModeManager(DEFAULT_FC_CONFIG),
  cliParser: new CLIParser(DEFAULT_FC_CONFIG),
  rcChannels: new Array(16).fill(0),
  lastRcUpdate: 0,
  loopDt: 1 / 2000,
  lastLoopTime: 0,
  loopCount: 0,
  cliBuffer: '',
  lastTelemSent: 0,
};

function controlLoop(): void {
  const now = performance.now();
  const dt = (now - engine.lastLoopTime) / 1000;
  engine.lastLoopTime = now;
  
  // Fixed timestep
  if (dt < engine.loopDt * 0.5) return; // Too early
  if (dt > engine.loopDt * 2) {
    // Too late - catch up
    engine.lastLoopTime = now;
  }
  
  const ps = engine.physicsState;
  const ci = engine.controlInput;
  
  if (ps.length === 0 || ci.length === 0) return;
  
  // Get sensor data
  const sensors = engine.sensorSim.getAll(ps);
  
  // Sensor fusion
  const orientation = engine.sensorFusion.update(sensors.gyro, sensors.accel, engine.loopDt, engine.config);
  
  // Flight mode
  const flightMode = engine.flightModeManager.update(engine.rcChannels, engine.loopDt);
  
  // Check armed state
  const armed = ci[CONTROL_OFFSETS.armed] > 0.5;
  
  // Throttle
  let throttle = engine.rcChannels[2] ?? 0; // Channel 3
  throttle = (throttle + 1) / 2; // -1..1 -> 0..1
  throttle = Math.max(0, Math.min(1, throttle));
  
  // Failsafe handling
  let rollRate = 0, pitchRate = 0, yawRate = 0;
  let rollAngle = 0, pitchAngle = 0, yawAngle = 0;
  
  if (engine.flightModeManager.isFailsafe()) {
    throttle = engine.flightModeManager.getFailsafeThrottle();
    rollRate = pitchRate = yawRate = 0;
    rollAngle = pitchAngle = yawAngle = 0;
  } else {
    // RC inputs (channels 0-3: roll, pitch, throttle, yaw)
    const rollStick = engine.rcChannels[0] ?? 0;
    const pitchStick = engine.rcChannels[1] ?? 0;
    const yawStick = engine.rcChannels[3] ?? 0;
    
    // Apply rates/expo (simplified)
    const rateScale = 2000 * DEG2RAD; // 2000 deg/s max
    const angleScale = 60 * DEG2RAD;  // 60 deg max
    
    rollRate = rollStick * rateScale;
    pitchRate = pitchStick * rateScale;
    yawRate = yawStick * rateScale;
    
    rollAngle = rollStick * angleScale;
    pitchAngle = pitchStick * angleScale;
    yawAngle = yawStick * angleScale;
  }
  
  // Measured rates (from gyro, body frame)
  const measuredRates = new Float32Array([
    sensors.gyro[0],
    sensors.gyro[1],
    sensors.gyro[2],
  ]);
  
  // Measured angles (from sensor fusion)
  const measuredAngles = new Float32Array(3);
  Quat.toEuler(measuredAngles, orientation);
  
  // PID computation
  let rollOut = 0, pitchOut = 0, yawOut = 0;
  
  if (flightMode === FLIGHT_MODE.ACRO || flightMode === FLIGHT_MODE.FAILSAFE) {
    // Rate mode
    rollOut = engine.pidController.compute(0, rollRate, measuredRates[0], throttle, engine.loopDt, true);
    pitchOut = engine.pidController.compute(1, pitchRate, measuredRates[1], throttle, engine.loopDt, true);
    yawOut = engine.pidController.compute(2, yawRate, measuredRates[2], throttle, engine.loopDt, true);
  } else if (flightMode === FLIGHT_MODE.ANGLE) {
    // Angle mode (cascaded: angle PID -> rate PID)
    const angleP = 50; // Angle P gain
    const maxAngle = engine.config.smallAngle * DEG2RAD;
    
    // Angle error
    const rollAngleError = Math.max(-maxAngle, Math.min(maxAngle, rollAngle)) - measuredAngles[0];
    const pitchAngleError = Math.max(-maxAngle, Math.min(maxAngle, pitchAngle)) - measuredAngles[1];
    
    // Angle PID output = rate setpoint
    const rollRateSp = angleP * rollAngleError;
    const pitchRateSp = angleP * pitchAngleError;
    
    // Rate PID
    rollOut = engine.pidController.compute(0, rollRateSp, measuredRates[0], throttle, engine.loopDt, true);
    pitchOut = engine.pidController.compute(1, pitchRateSp, measuredRates[1], throttle, engine.loopDt, true);
    yawOut = engine.pidController.compute(2, yawRate, measuredRates[2], throttle, engine.loopDt, true);
  } else if (flightMode === FLIGHT_MODE.HORIZON) {
    // Horizon mode (blend angle and rate)
    // Simplified: just use angle mode for now
    const angleP = 40;
    const maxAngle = engine.config.levelAngleLimit * DEG2RAD;
    
    const rollAngleError = Math.max(-maxAngle, Math.min(maxAngle, rollAngle)) - measuredAngles[0];
    const pitchAngleError = Math.max(-maxAngle, Math.min(maxAngle, pitchAngle)) - measuredAngles[1];
    
    const rollRateSp = angleP * rollAngleError;
    const pitchRateSp = angleP * pitchAngleError;
    
    rollOut = engine.pidController.compute(0, rollRateSp, measuredRates[0], throttle, engine.loopDt, true);
    pitchOut = engine.pidController.compute(1, pitchRateSp, measuredRates[1], throttle, engine.loopDt, true);
    yawOut = engine.pidController.compute(2, yawRate, measuredRates[2], throttle, engine.loopDt, true);
  }
  
  // Mixer
  const motorOutputs = engine.mixer.mix(throttle, rollOut, pitchOut, yawOut);
  
  // Apply motor limits
  const motorCount = engine.mixer.getMotorCount();
  for (let i = 0; i < motorCount; i++) {
    motorOutputs[i] = Math.max(engine.config.motorIdle, Math.min(1, motorOutputs[i]));
    ci[CONTROL_OFFSETS.motorCmd0 + i] = motorOutputs[i];
  }
  
  // Update control input shared memory
  ci[CONTROL_OFFSETS.throttle] = throttle;
  ci[CONTROL_OFFSETS.targetRateX] = rollRate;
  ci[CONTROL_OFFSETS.targetRateY] = pitchRate;
  ci[CONTROL_OFFSETS.targetRateZ] = yawRate;
  ci[CONTROL_OFFSETS.targetAngleX] = rollAngle;
  ci[CONTROL_OFFSETS.targetAngleY] = pitchAngle;
  ci[CONTROL_OFFSETS.targetAngleZ] = yawAngle;
  ci[CONTROL_OFFSETS.flightMode] = flightMode;
  ci[CONTROL_OFFSETS.armed] = armed ? 1 : 0;
  ci[CONTROL_OFFSETS.timestamp] = now;
  ci[CONTROL_OFFSETS.sequence] = engine.loopCount++;
  
  // Send telemetry occasionally
  if (engine.loopCount % 200 === 0) { // 10 Hz at 2kHz
    (self as any).postMessage({
      type: 'telemetry',
      data: {
        loopCount: engine.loopCount,
        flightMode,
        armed,
        throttle,
        orientation: [orientation[0], orientation[1], orientation[2], orientation[3]],
        rates: [measuredRates[0], measuredRates[1], measuredRates[2]],
        angles: [measuredAngles[0], measuredAngles[1], measuredAngles[2]],
        pidOutput: [rollOut, pitchOut, yawOut],
        motorOutputs,
        iterms: [
          engine.pidController.getITerm(0),
          engine.pidController.getITerm(1),
          engine.pidController.getITerm(2),
        ],
        batteryVoltage: ps[PHYSICS_STATE_OFFSETS.batteryVoltage],
      },
    });
  }
}

// Worker message handler
(self as any).onmessage = (event: MessageEvent) => {
  const message = event.data;
  
  switch (message.type) {
    case 'init':
      engine.config = { ...engine.config, ...message.config };
      engine.physicsState = new Float32Array(message.physicsBuffer);
      engine.controlInput = new Float32Array(message.controlBuffer);
      
      // Reinitialize components
      engine.sensorSim = new SensorSimulator(engine.config);
      engine.sensorFusion = new SensorFusion();
      engine.pidController = new PIDController(engine.config);
      engine.mixer = new Mixer(engine.config);
      engine.flightModeManager = new FlightModeManager(engine.config);
      engine.cliParser = new CLIParser(engine.config);
      
      engine.loopDt = 1 / engine.config.loopRate;
      engine.lastLoopTime = performance.now();
      engine.loopCount = 0;
      
      (self as any).postMessage({ type: 'ready' });
      break;
      
    case 'rcInput':
      // RC channels: array of 16 values (-1 to 1)
      if (message.channels && message.channels.length >= 16) {
        engine.rcChannels = [...message.channels];
        engine.lastRcUpdate = performance.now();
      }
      break;
      
    case 'cli':
      // CLI command
      const result = engine.cliParser.parse(message.command);
      (self as any).postMessage({
        type: 'cliResponse',
        success: result.success,
        response: result.response,
      });
      // Apply config changes
      if (message.command.startsWith('set ')) {
        engine.sensorSim.updateConfig(engine.config);
        engine.pidController.updateConfig(engine.config);
        engine.mixer.updateConfig(engine.config);
        engine.flightModeManager.updateConfig(engine.config);
      }
      break;
      
    case 'setConfig':
      engine.config = { ...engine.config, ...message.config };
      engine.sensorSim.updateConfig(engine.config);
      engine.pidController.updateConfig(engine.config);
      engine.mixer.updateConfig(engine.config);
      engine.flightModeManager.updateConfig(engine.config);
      engine.cliParser = new CLIParser(engine.config);
      engine.loopDt = 1 / engine.config.loopRate;
      break;
      
    case 'setPidProfile':
      engine.pidController.setProfile(message.profileIndex);
      break;
      
    case 'reset':
      engine.sensorFusion.reset();
      engine.pidController.reset();
      engine.loopCount = 0;
      break;
      
    case 'pause':
      // Not needed - controlled by main thread
      break;
      
    case 'resume':
      engine.lastLoopTime = performance.now();
      break;
      
    default:
      console.warn('[FCWorker] Unknown message:', message.type);
  }
};

// Main loop
setInterval(controlLoop, 0); // Run as fast as possible, controlled by timing inside

export {};