# FPV Drone Simulator — Architectural Design Document

**Project:** fpv-drone-simulator  
**Author:** Marcus Vance (Lead Systems & Backend Architect)  
**Date:** 2026-11-23  
**Status:** INITIAL DESIGN — Review Required  
**Reference Specification:** `references/Betaflight-2026-flight-sim-trainer.md`

---

## 1. Executive Summary

This document defines the complete system architecture for a browser-based FPV drone simulator and pilot-training platform that achieves **Betaflight-compatible flight-controller behavior**, **credible quadcopter physics**, **real transmitter/gamepad compatibility**, **high-quality real-time 3D graphics**, **realistic FPV camera presentation**, **structured pilot training**, **flight telemetry/recording/analysis**, **racing/freestyle/multiplayer capabilities**, and **persistent pilot profiles** — all running in a modern browser with visual quality rivaling native PC games.

**Core Philosophy:** Engineering credibility first, visual spectacle as a consequence of physical accuracy. Every visual effect must be grounded in simulated physics; no "fake" camera shake or artificial motion blur. The simulator must be a legitimate training instrument that happens to look exceptional.

---

## 2. System Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           BROWSER RUNTIME (Main Thread)                     │
├─────────────────────────────────────────────────────────────────────────────┤
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐    │
│  │   Input      │  │   UI/UX      │  │   Graphics   │  │   Audio      │    │
│  │   Manager    │  │   Layer      │  │   Engine     │  │   Engine     │    │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘    │
│         │                 │                 │                 │            │
│         └─────────────────┼─────────────────┼─────────────────┘            │
│                           ▼                                                 │
│              ┌────────────────────────────┐                               │
│              │   Application Coordinator   │                               │
│              │   (State Machine + Router)  │                               │
│              └──────────────┬──────────────┘                               │
│                             │                                               │
└─────────────────────────────┼───────────────────────────────────────────────┘
                              │
              ┌───────────────┼───────────────┐
              ▼               ▼               ▼
       ┌────────────┐ ┌────────────┐ ┌────────────┐
       │  Physics   │ │  Flight    │ │  Training  │
       │  Worker    │ │  Controller│ │  Worker    │
       │ (WebWorker)│ │  Worker    │ │ (WebWorker)│
       └────────────┘ └────────────┘ └────────────┘
              │               │               │
              └───────────────┼───────────────┘
                              ▼
                    ┌───────────────────┐
                    │  SharedArrayBuffer│
                    │  (Physics State)  │
                    └───────────────────┘
```

### 2.1 Threading Model

| Thread | Responsibility | Communication |
|--------|---------------|---------------|
| **Main Thread** | Rendering, UI, Input polling, Audio, Coordinator | postMessage, SharedArrayBuffer |
| **Physics Worker** | Rigid-body dynamics, motor model, aerodynamics, integration (1kHz fixed-step) | SharedArrayBuffer (read/write), postMessage (events) |
| **Flight Controller Worker** | Betaflight PID loops, mixer, modes, failsafe (2kHz/4kHz) | SharedArrayBuffer (read/write), postMessage (commands) |
| **Training Worker** | Maneuver detection, scoring, progression, ghost replay | postMessage (async) |
| **Telemetry Worker** | Data logging, compression, export, analysis | postMessage (async) |
| **Network Worker** | WebRTC signaling, state synchronization, rollback netcode | postMessage (async) |

**Why Web Workers?** Physics and flight controller must run at fixed, high frequencies (1–4 kHz) independent of frame rate. Main thread at 60–144 Hz cannot guarantee deterministic integration. SharedArrayBuffer with Atomics enables lock-free state sharing between workers and main thread for rendering.

---

## 3. Core Physics Engine

### 3.1 Coordinate Systems

```
World Frame (NED):     North-East-Down, origin at takeoff point
Body Frame (FLU):      Forward-Left-Up, origin at center of mass
Camera Frame:          Aligned with FPV camera mount (configurable offset)
```

**Convention:** Right-handed, NED for world (standard in aviation), FLU for body (standard in robotics). Transformations use quaternions exclusively — no Euler angles in simulation core.

### 3.2 Quadcopter Rigid-Body Dynamics

**State Vector (13 DOF):**
```
Position:       p = [x, y, z]           (world NED, meters)
Velocity:       v = [vx, vy, vz]        (world NED, m/s)
Orientation:    q = [w, x, y, z]        (unit quaternion, world→body)
Angular Vel:    ω = [ωx, ωy, ωz]        (body frame, rad/s)
```

**Equations of Motion (Newton-Euler):**
```
ṗ = v
v̇ = g + (1/m) * R(q) * F_body
q̇ = 0.5 * q ⊗ [0, ω]
ω̇ = I⁻¹ * (τ_body - ω × (I * ω))
```

Where:
- `g = [0, 0, 9.80665]` (NED, positive down)
- `R(q)` = rotation matrix world←body
- `F_body` = total force in body frame (thrust + drag)
- `τ_body` = total torque in body frame (motor torques + aerodynamic)
- `I` = 3×3 inertia tensor (diagonal for symmetric quads)

### 3.3 Motor Model

**Brushless DC Motor (BLDC) First-Order Model:**
```
τ_motor * dω_motor/dt = ω_cmd - ω_motor
thrust = k_t * ω_motor²
torque = k_q * ω_motor²
```

**Parameters (per motor, configurable per quad profile):**
| Parameter | Symbol | Typical Value | Unit |
|-----------|--------|---------------|------|
| Thrust coefficient | k_t | 1.2e-5 | N/(rad/s)² |
| Drag coefficient | k_q | 2.4e-7 | Nm/(rad/s)² |
| Motor time constant | τ_motor | 0.02 | s |
| Max RPM | ω_max | 32000 | RPM |
| Propeller inertia | J_prop | 5e-6 | kg·m² |

**Voltage Sag Model:**
```
V_actual = V_nominal - I_total * R_internal - k_sag * ∫I_total dt
ω_cmd = clamp(throttle * V_actual / V_nominal, 0, 1) * ω_max
```

### 3.4 Aerodynamics

**Body-Frame Drag (quadratic):**
```
F_drag = -0.5 * ρ * v_body² * C_d * A_ref * sign(v_body)
```

**Propeller Wash / Ground Effect:**
```
F_ground = k_ground * (rotor_disk_area / (4π * z²)) * thrust  (z < 1.5 * prop_diameter)
```

**Wind Model (configurable):**
```
wind_world = wind_steady + wind_gust(t) + wind_shear(z)
wind_body = R(q)ᵀ * wind_world
v_relative = v_body - wind_body
```

### 3.5 Integration Scheme

**Method:** 4th-order Runge-Kutta (RK4) with fixed timestep Δt = 1 ms (1 kHz)
- Deterministic, symplectic properties for energy conservation
- Sub-stepping for motor dynamics (τ_motor ≈ 20 ms → 20 sub-steps per physics tick)
- State checkpointing for rollback netcode (multiplayer)

**Pseudo-code:**
```javascript
function physicsStep(state, controls, dt) {
  // RK4 integration
  const k1 = derivatives(state, controls);
  const k2 = derivatives(state + 0.5*dt*k1, controls);
  const k3 = derivatives(state + 0.5*dt*k2, controls);
  const k4 = derivatives(state + dt*k3, controls);
  return state + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);
}
```

---

## 4. Betaflight-Compatible Flight Controller

### 4.1 Architecture

The flight controller (FC) runs as a **separate WebWorker at 2–4 kHz** (configurable loop rate), consuming sensor data from the physics engine and producing motor commands. This mirrors real Betaflight's separation of sensor fusion (gyro/accel) and PID control.

### 4.2 Sensor Simulation

**Gyroscope (3-axis):**
```
ω_measured = ω_true + bias + noise + temp_drift
bias = bias_0 + random_walk(σ_bias)
noise ~ N(0, σ_gyro²)  (σ_gyro ≈ 0.005 rad/s/√Hz for ICM-42688)
```

**Accelerometer (3-axis):**
```
a_measured = R(q)ᵀ * (g - v̇) + bias + noise + vibe
vibe = motor_harmonics(ω_motor)  // RPM-synchronous vibration
```

**Barometer:**
```
pressure = P₀ * (1 - L*z/T₀)^(g*M/R*L) + noise
altitude = (T₀/L) * (1 - (pressure/P₀)^(R*L/g*M))
```

**Magnetometer:**
```
mag_measured = R(q)ᵀ * mag_earth + hard_iron + soft_iron * mag_earth + noise
```

### 4.3 Sensor Fusion (INS)

**Complementary Filter (Betaflight-style):**
```
q_fused = slerp(q_gyro, q_accel_mag, α)
α = 0.98  // Gyro trust factor (configurable)
```

**Mahony Filter (Optional, for GPS-assisted):**
```
q̇ = 0.5 * q ⊗ [0, ω_measured - k_p * e - k_i * ∫e dt]
e = (a_measured × g_est) + (m_measured × m_est)  // Error vector
```

### 4.4 PID Controller (Betaflight 4.5+ Compatible)

**Axis Structure (per axis: roll, pitch, yaw):**
```
P_term = Kp * error
I_term += Ki * error * dt  (with anti-windup: clamp, decay, setpoint weighting)
D_term = Kd * (error - error_prev) / dt  (derivative on measurement, not error)
FF_term = Kff * setpoint_rate  // Feedforward
output = P_term + I_term + D_term + FF_term
```

**Betaflight-Specific Features:**
- **Setpoint Weighting:** `error = setpoint_weight * setpoint - measurement`
- **D-term Filter:** 2nd-order low-pass (configurable cutoff, default 30 Hz)
- **ITerm Relax:** Reduce I-term when throttle near 0 or max
- **Anti-Gravity:** Boost I-term on rapid throttle changes
- **Dynamic Idle:** Adjust motor idle based on thrust demand
- **RPM Filter:** Notch filter at motor RPM harmonics (requires RPM telemetry)

**PID Profiles (switchable in-flight):**
- Profile 0: Race (high P, low I, aggressive D)
- Profile 1: Freestyle (moderate P, higher I, smooth D)
- Profile 2: Cinematic (low P, high I, heavy filtering)
- Profile 3: Custom (user-defined)

### 4.5 Mixer

**Motor Layouts (configurable):**
```
Quad X:     [+1, -1, +1, -1]  roll
            [-1, -1, +1, +1]  pitch
            [-1, +1, -1, +1]  yaw
            
Quad H:     [+1, -1, +1, -1]  roll (wide)
            [-1, -1, +1, +1]  pitch (long)
            [-1, +1, -1, +1]  yaw
            
Hex X:      6 motors, 60° spacing
Octo X:     8 motors, 45° spacing
```

**Mixer Output:**
```
motor[i] = throttle + roll_mix[i]*roll_out + pitch_mix[i]*pitch_out + yaw_mix[i]*yaw_out
motor[i] = clamp(motor[i], motor_idle, motor_max)
```

**Output Protocols:** DShot300/600/1200, Multishot, Oneshot125 (simulated timing)

### 4.6 Flight Modes (Betaflight Parity)

| Mode | Description | Activation |
|------|-------------|------------|
| ACRO | Rate mode, no self-level | Default |
| ANGLE | Self-level with max angle limit | Switch |
| HORIZON | Angle + acro blend | Switch |
| AIR | Acro with auto-level on stick center | Switch |
| FAILSAFE | Configured behavior (drop, hover, RTH) | Signal loss |
| GPS_RESCUE | Return to home (requires GPS sim) | Switch/Failsafe |

### 4.7 Configuration System (CLI + GUI)

**Betaflight CLI Commands (subset):**
```
set pid_profile_0_pid = 50, 50, 50, 50, 50, 50
set pid_profile_0_dterm_filter_type = 1
set pid_profile_0_dterm_filter_hz = 30
set mixer = QUADX
set motor_pwm_protocol = DSHOT600
set gyro_lpf = 1
set gyro_soft_lpf = 1
set gyro_lpf_hz = 200
save
```

**Persistence:** IndexedDB (local) + cloud sync (optional)

---

## 5. Input System

### 5.1 Real Transmitter Support

**Protocols:**
- **USB HID Joystick:** FrSky Taranis, RadioMaster, Jumper, FlySky (native HID)
- **Crossfire/ELRS CRSF:** Via USB serial (Web Serial API) — requires user permission
- **TBS Tracer:** USB HID + proprietary protocol
- **ExpressLRS Joystick:** USB HID

**Implementation:**
```javascript
// Gamepad API (standard transmitters)
navigator.getGamepads()
// Web Serial API (CRSF, MSP)
const port = await navigator.serial.requestPort();
await port.open({ baudRate: 420000 });
```

**Channel Mapping (configurable per transmitter):**
```
Channel 1: Roll     (Aileron)
Channel 2: Pitch    (Elevator)
Channel 3: Throttle
Channel 4: Yaw      (Rudder)
Channel 5: Mode     (Arm/Disarm, Flight Mode)
Channel 6: Aux 1    (Beeper, Turtle, etc.)
Channel 7: Aux 2    (FPV Camera Angle, etc.)
Channel 8: Aux 3    (Custom)
...
Channel 16: Aux 12
```

**Calibration Wizard:**
- Center/endpoint detection per channel
- Deadband configuration
- Expo/Rate curves (Betaflight-style: `rate = expo * (stick^3) + (1-expo) * stick`)

### 5.2 Gamepad Support

**Standard Gamepads:** Xbox, PlayStation, Steam Deck, 8BitDo
- Left stick: Pitch/Roll (Mode 2) or Roll/Yaw (Mode 1)
- Right stick: Throttle/Yaw (Mode 2) or Pitch/Throttle (Mode 1)
- Triggers: Aux channels
- D-pad: Mode switches

**Virtual Sticks (Touch/Mouse):** On-screen joysticks for mobile/desktop fallback

### 5.3 Input Processing Pipeline

```
Raw Input → Calibration → Deadband → Expo/Rate → Channel Mapping → FC Input
                    ↓
              Smoothing (optional, configurable latency vs. smoothness)
```

---

## 6. Graphics Engine

### 6.1 Rendering Architecture

**API:** WebGPU (primary) with WebGL 2 fallback
- Compute shaders for particle systems, grass, procedural terrain
- Storage buffers for instanced rendering (thousands of objects)
- Bindless resources for flexible material system

**Render Pipeline (Deferred + Forward):**
```
1. Shadow Map Pass (cascaded, 4 splits)
2. G-Buffer Pass (Position, Normal, Albedo, Material, Velocity)
3. SSAO Pass (Horizon-based, temporal)
4. Lighting Pass (Clustered forward, 256 lights max)
5. Sky/Atmosphere Pass (Precomputed Bruneton, procedural clouds)
6. Transparent Pass (Water, Glass, Particles - depth-sorted)
7. FPV Camera Pass (Post-process stack)
8. UI Pass (Immediate-mode, WebGPU compute)
```

### 6.2 FPV Camera Simulation

**Physical Camera Model:**
```
Sensor:        Configurable (4:3, 16:9, 5:4), resolution, pixel pitch
Lens:          FOV (90°–170°), distortion (barrel/fisheye), vignetting
Mount:         Position offset, angle offset, vibration transfer function
```

**FPV-Specific Effects (All Physics-Driven):**
- **Camera Tilt:** Animated servo (configurable speed, 0°–60°)
- **Vibration:** Motor harmonics → mount transfer function → sensor shake (not post-process!)
- **Rolling Shutter:** Row-by-row exposure simulation (configurable readout time)
- **Exposure/Auto-Gain:** Simulated CMOS response, highlights blow out
- **Latency:** Configurable (1–50 ms) — adds to control loop latency
- **Static/Noise:** Analog-style breakup at signal edge (optional)
- **OSD Overlay:** Betaflight-style OSD elements (battery, RSSI, timer, GPS)

**Post-Process Stack (FPV Pass):**
```
1. Lens Distortion (Brown-Conrady model)
2. Chromatic Aberration (radial, configurable)
3. Vignetting (cos⁴ falloff)
4. Film Grain (temporal, ISO-dependent)
5. Color Grade (LUT, configurable: analog, digital, custom)
6. Motion Blur (velocity buffer, shutter angle)
7. Bloom (threshold, knee, intensity)
8. Tonemap (ACES, Reinhard, or custom)
9. Dither (8-bit banding reduction)
```

### 6.3 Environment System

**Terrain:**
- **Procedural:** Infinite, seeded, GPU-generated (compute shader)
- **Biomes:** Field, Forest, Urban, Warehouse, Race Track, Mountain
- **LOD:** Tessellation + geometry clipmaps (WebGPU) or manual LOD (WebGL)
- **Collision:** Heightfield sampled in physics worker (SharedArrayBuffer)

**Vegetation:**
- **Grass:** Compute shader, 100k+ blades, wind animation, interaction
- **Trees:** Instanced, LOD billboards → high-poly, procedural placement
- **Props:** Gates, flags, tires, barrels — physics-enabled (kinematic)

**Atmosphere:**
- **Sky:** Precomputed atmospheric scattering (Bruneton), time-of-day cycle
- **Clouds:** 3D volumetric (ray-marched), weather system
- **Fog:** Exponential + height fog, volumetric light shafts
- **Precipitation:** Rain/snow particles (compute, GPU-simulated)

### 6.4 Drone Rendering

**Model:** GLTF 2.0, PBR materials (metallic/roughness)
- **Components:** Frame, Motors, Props (spinning disc + motion blur), Camera, Antennas, Battery, Receiver, ESC
- **Damage System:** Deformable frame (vertex shader), broken props, bent motors
- **LED Strips:** Addressable RGB (Betaflight LED strip simulation)

**Propeller Rendering:**
```
Low RPM:    High-poly mesh, motion blur
High RPM:   Disc shader (alpha-blended, refractive, strobe effect)
Transition: Smooth crossfade at ~2000 RPM
```

---

## 7. Training System

### 7.1 Maneuver Definition

**Data-Driven Maneuvers (JSON + WebAssembly validation):**
```json
{
  "id": "split-s",
  "name": "Split-S",
  "category": "freestyle",
  "difficulty": 3,
  "description": "Half roll + half loop, descending 180° direction change",
  "phases": [
    { "name": "entry", "duration": 1.0, "constraints": { "roll_rate": ">300", "throttle": "<0.3" }},
    { "name": "inverted", "duration": 0.5, "constraints": { "pitch": "<-80", "roll": "≈180" }},
    { "name": "recovery", "duration": 1.5, "constraints": { "pitch": ">10", "roll_rate": "<50" }}
  ],
  "scoring": {
    "time_weight": 0.3,
    "smoothness_weight": 0.4,
    "accuracy_weight": 0.3,
    "bonus": { "perfect_entry": 100, "no_wobble": 50 }
  },
  "ghost_reference": "trajectories/split-s.bin"
}
```

### 7.2 Training Modes

| Mode | Description |
|------|-------------|
| **Free Flight** | No objectives, telemetry recording only |
| **Maneuver Practice** | Single maneuver, ghost replay, step-by-step coaching |
| **Course** | Sequence of gates/maneuvers, timed, scored |
| **Scenario** | Scripted situation (e.g., "recover from flip", "navigate tree gap") |
| **Multiplayer Race** | Synced start, ghost opponents, leaderboard |
| **Freestyle Session** | Scored trick combo detection, flow analysis |

### 7.3 Progression System

**Pilot Profile (persistent, IndexedDB + optional cloud):**
```typescript
interface PilotProfile {
  id: string;
  name: string;
  created: Date;
  stats: {
    totalFlightTime: number;      // seconds
    totalDistance: number;        // meters
    maneuversCompleted: Map<string, ManeuverStat>;
    coursesCompleted: Map<string, CourseStat>;
    highestScore: number;
    rank: 'Novice' | 'Beginner' | 'Intermediate' | 'Advanced' | 'Expert' | 'Pro';
  };
  preferences: {
    controlMode: 1 | 2;
    rates: RateProfile;
    cameraTilt: number;
    fov: number;
    graphicsPreset: 'low' | 'medium' | 'high' | 'ultra';
  };
  quadProfiles: QuadProfile[];
  ghosts: GhostReplay[];
  achievements: Achievement[];
}
```

**Maneuver Stat:**
```typescript
interface ManeuverStat {
  maneuverId: string;
  attempts: number;
  bestScore: number;
  bestTime: number;
  averageScore: number;
  lastAttempt: Date;
  stars: 0 | 1 | 2 | 3;  // 3 = mastered
  ghostId?: string;
}
```

### 7.4 Ghost Replay System

- **Recording:** Sample state at 60 Hz (position, orientation, motor outputs, stick inputs)
- **Compression:** Delta encoding + quantization (≤ 50 KB/min)
- **Playback:** Interpolated rendering, synchronized with physics
- **Comparison:** Side-by-side or overlay with delta visualization

---

## 8. Telemetry & Recording

### 8.1 Data Streams

| Stream | Rate | Content |
|--------|------|---------|
| **High-Rate** | 1–4 kHz | Full state (pos, vel, quat, ω, motor RPM, PID terms, sensor raw) |
| **Standard** | 60 Hz | Position, orientation, throttle, stick inputs, mode, flags |
| **Low-Rate** | 1 Hz | Battery, RSSI, GPS, temperatures, statistics |
| **Event** | Async | Arm/disarm, crash, mode change, failsafe, lap crossing |

### 8.2 Recording Format

**Binary Format (custom, optimized):**
```
Header: magic(4) + version(1) + timestamp(8) + profile_hash(16) + config_json_len(4) + config_json
Frames: [timestamp(4) + frame_type(1) + payload(var)]...
Index:  frame_offsets + frame_timestamps (for random access)
```

**Export Formats:**
- **.fpvlog** (native binary)
- **CSV** (standard telemetry)
- **KML/GPX** (Google Earth, GPS visualizers)
- **Blackbox** (Betaflight-compatible log format)
- **JSON** (web analysis tools)

### 8.3 Analysis Tools

**Built-in (Web-based):**
- 3D trajectory viewer (WebGPU)
- PID tuning analyzer (FFT, step response, oscillation detection)
- Motor/ESC performance (RPM vs. thrust, thermal)
- Battery discharge curves
- Rate/Expo curve editor with live preview
- Stick input heatmaps

**External Compatibility:**
- Betaflight Configurator (Blackbox tab)
- Blackbox Explorer
- FPV Log Viewer
- Custom Python/Matlab scripts (via CSV/JSON export)

---

## 9. Multiplayer Architecture

### 9.1 Network Model

**Architecture:** Client-authoritative with server reconciliation (rollback netcode)
- **Transport:** WebRTC DataChannels (UDP-like, unreliable + reliable)
- **Signaling:** WebSocket server (minimal: session creation, ICE exchange)
- **Topology:** Mesh (≤8 pilots) or Star (server-authoritative for >8)

### 9.2 State Synchronization

**Deterministic Simulation:**
- Fixed physics timestep (1 ms) on all clients
- Identical inputs → identical outputs (floating-point determinism via `fesetround` or fixed-point)
- Input frames timestamped, buffered, executed in lockstep

**Rollback Protocol (GGPO-style):**
```
1. Local prediction: simulate ahead with local inputs
2. Remote inputs arrive: if past frame → rollback to that frame, re-simulate
3. Correction: blend corrected state into predicted state (smooth)
4. Confirmed frames: garbage collected
```

**Bandwidth Optimization:**
- Input-only transmission (4 bytes/frame: 4 channels × 10 bits)
- State snapshots every 60 frames (1 second) for late joiners
- Delta compression for spectator streams

### 9.3 Session Types

| Type | Max Pilots | Authority | Use Case |
|------|------------|-----------|----------|
| **Race** | 8 | Server (lap timing) | Competition |
| **Freestyle Jam** | 16 | Client (trick detection) | Practice session |
| **Training** | 4 | Server (instructor) | Coached session |
| **Spectator** | 100 | Read-only | Streaming |

---

## 10. Persistent Data & Backend

### 10.1 Local-First Architecture

All data lives in **IndexedDB** (browser) with optional **cloud sync** to a simple REST API.

**IndexedDB Schema:**
```
pilotProfiles    (key: profileId)
quadProfiles     (key: quadId)
maneuverDefs     (key: maneuverId)
courseDefs       (key: courseId)
flightLogs       (key: logId, index: timestamp)
ghostReplays     (key: ghostId, index: maneuverId)
settings         (key: 'global')
```

### 10.2 Cloud Backend (Optional, Minimal)

**API Endpoints (REST, PHP/MySQL on Hostinger):**
```
POST   /api/v1/sync/profile     → Upsert pilot profile
GET    /api/v1/sync/profile/:id → Fetch profile
POST   /api/v1/sync/log         → Upload flight log
GET    /api/v1/leaderboard/:courseId → Top 100
POST   /api/v1/ghost            → Upload ghost
GET    /api/v1/ghost/:maneuverId → Top ghosts for maneuver
```

**Authentication:** JWT (HS256), short-lived access + refresh tokens
**Storage:** MySQL (profiles, leaderboards) + S3-compatible (logs, ghosts)

### 10.3 Data Export/Import

- Full profile backup (JSON + binary logs)
- Selective export (specific logs, quads, ghosts)
- Import with conflict resolution (merge, replace, rename)

---

## 11. Project Structure

```
fpv-drone-simulator/
├── public/
│   ├── index.html                 # Entry point
│   ├── manifest.json              # PWA manifest
│   ├── sw.js                      # Service worker (offline)
│   ├── assets/
│   │   ├── models/                # GLTF drone models, props
│   │   ├── textures/              # PBR materials, terrain textures
│   │   ├── shaders/               # WGSL/GLSL shaders
│   │   ├── audio/                 # Motor sounds, wind, impacts
│   │   └── data/                  # Maneuver defs, courses, default profiles
│   └── wasm/                      # Compiled WASM modules
├── src/
│   ├── main.ts                    # Application bootstrap
│   ├── coordinator/
│   │   ├── ApplicationCoordinator.ts
│   │   ├── StateMachine.ts
│   │   └── Router.ts
│   ├── core/
│   │   ├── physics/
│   │   │   ├── PhysicsWorker.ts
│   │   │   ├── RigidBody.ts
│   │   │   ├── MotorModel.ts
│   │   │   ├── Aerodynamics.ts
│   │   │   ├── Integrator.ts
│   │   │   └── WindModel.ts
│   │   ├── flight-controller/
│   │   │   ├── FCWorker.ts
│   │   │   ├── SensorSim.ts
│   │   │   ├── SensorFusion.ts
│   │   │   ├── PIDController.ts
│   │   │   ├── Mixer.ts
│   │   │   ├── FlightModes.ts
│   │   │   └── CLIParser.ts
│   │   ├── math/
│   │   │   ├── Vec3.ts
│   │   │   ├── Quat.ts
│   │   │   ├── Mat3.ts
│   │   │   └── Random.ts
│   │   └── config/
│   │       ├── QuadProfile.ts
│   │       ├── RateProfile.ts
│   │       └── PIDProfile.ts
│   ├── graphics/
│   │   ├── Renderer.ts
│   │   ├── Scene.ts
│   │   ├── Camera.ts
│   │   ├── FPVCamera.ts
│   │   ├── PostProcess.ts
│   │   ├── Terrain.ts
│   │   ├── Vegetation.ts
│   │   ├── Atmosphere.ts
│   │   ├── DroneRenderer.ts
│   │   ├── Lighting.ts
│   │   └── shaders/               # WGSL source
│   ├── input/
│   │   ├── InputManager.ts
│   │   ├── Transmitter.ts
│   │   ├── Gamepad.ts
│   │   ├── VirtualSticks.ts
│   │   ├── Calibration.ts
│   │   └── WebSerial.ts
│   ├── training/
│   │   ├── TrainingWorker.ts
│   │   ├── ManeuverEngine.ts
│   │   ├── CourseEngine.ts
│   │   ├── GhostSystem.ts
│   │   ├── Progression.ts
│   │   └── Coach.ts
│   ├── telemetry/
│   │   ├── TelemetryWorker.ts
│   │   ├── Recorder.ts
│   │   ├── Exporter.ts
│   │   ├── Analyzer.ts
│   │   └── BlackboxCompat.ts
│   ├── multiplayer/
│   │   ├── NetworkWorker.ts
│   │   ├── WebRTCManager.ts
│   │   ├── RollbackNetcode.ts
│   │   ├── SignalingClient.ts
│   │   └── SessionManager.ts
│   ├── persistence/
│   │   ├── IndexedDB.ts
│   │   ├── CloudSync.ts
│   │   ├── ProfileManager.ts
│   │   └── Migration.ts
│   ├── ui/
│   │   ├── UIManager.ts
│   │   ├── HUD.ts
│   │   ├── Menus.ts
│   │   ├── Settings.ts
│   │   ├── OSD.ts
│   │   └── CalibrationWizard.ts
│   ├── audio/
│   │   ├── AudioEngine.ts
│   │   ├── MotorSound.ts
│   │   ├── WindSound.ts
│   │   └── SpatialAudio.ts
│   └── utils/
│       ├── WorkerPool.ts
│       ├── SharedBuffer.ts
│       ├── Performance.ts
│       └── Debug.ts
├── workers/
│   ├── physics.worker.ts
│   ├── fc.worker.ts
│   ├── training.worker.ts
│   ├── telemetry.worker.ts
│   └── network.worker.ts
├── tests/
│   ├── unit/
│   ├── integration/
│   ├── physics/
│   └── fc/
├── docs/
│   ├── ARCHITECTURE.md            # This file
│   ├── PHYSICS.md
│   ├── FLIGHT_CONTROLLER.md
│   ├── GRAPHICS.md
│   ├── INPUT.md
│   ├── TRAINING.md
│   ├── TELEMETRY.md
│   ├── MULTIPLAYER.md
│   └── DEPLOYMENT.md
├── package.json
├── tsconfig.json
├── vite.config.ts
├── .gitignore
├── LICENSE
└── README.md
```

---

## 12. Technology Stack

| Layer | Technology | Version | Rationale |
|-------|------------|---------|-----------|
| **Language** | TypeScript | 5.6+ | Type safety, worker communication, IDE support |
| **Build** | Vite | 5.x | Fast HMR, WebWorker/WASM support, PWA plugin |
| **Graphics API** | WebGPU | W3C Standard | Compute shaders, storage buffers, bindless, modern |
| **Graphics Fallback** | WebGL 2 | W3C Standard | Universal compatibility |
| **Math** | gl-matrix + custom | 3.4+ | SIMD-optimized, no allocation in hot paths |
| **Physics Worker** | TypeScript → WASM (optional) | — | Future: compile hot paths to WASM for perf |
| **3D Format** | GLTF 2.0 | Khronos | Industry standard, PBR, animations |
| **Audio** | Web Audio API | W3C Standard | Spatial, real-time synthesis |
| **Input** | Gamepad API + Web Serial API | W3C Standard | Transmitter support |
| **Storage** | IndexedDB (idb wrapper) | W3C Standard | Large, structured, transactional |
| **Networking** | WebRTC DataChannels | W3C Standard | UDP-like, NAT traversal, encryption |
| **Signaling** | WebSocket (ws library) | — | Minimal server component |
| **Testing** | Vitest + Playwright | — | Unit + E2E |
| **Linting** | ESLint + TypeScript ESLint | — | Code quality |
| **Formatting** | Prettier | — | Consistency |
| **CI/CD** | GitHub Actions | — | Automated test, build, deploy |

---

## 13. Performance Targets

| Metric | Target | Measurement |
|--------|--------|-------------|
| **Physics Timestep** | 1 ms (1 kHz) fixed | Deterministic |
| **FC Loop Rate** | 2–4 kHz configurable | Deterministic |
| **Frame Rate** | 60–144 FPS (VRR) | Main thread |
| **Frame Time (Main)** | < 8 ms (120 FPS) | Chrome DevTools |
| **Frame Time (Graphics)** | < 6 ms (Ultra) | GPU Profiler |
| **Worker Latency** | < 0.5 ms | postMessage round-trip |
| **Memory (Total)** | < 512 MB | Chrome Task Manager |
| **Memory (Workers)** | < 128 MB each | Chrome Task Manager |
| **Cold Load** | < 3 s (cached) | Lighthouse |
| **Cold Load** | < 8 s (uncached) | Lighthouse |
| **Input Latency** | < 20 ms (stick→screen) | High-speed camera |
| **Network RTT** | < 100 ms (playable) | WebRTC stats |
| **Rollback Frames** | ≤ 3 frames typical | Netcode stats |

**Quality Presets:**
- **Low:** 30 FPS target, reduced LOD, no SSAO, simple shaders
- **Medium:** 60 FPS, SSAO, basic volumetric
- **High:** 90 FPS, full volumetric, 4k shadow maps
- **Ultra:** 120+ FPS, ray-traced shadows (WebGPU), 8k shadows, max particles

---

## 14. Development Phases

### Phase 1: Foundation (Weeks 1–2)
- [ ] Repository setup, CI/CD, TypeScript config, Vite config
- [ ] Worker infrastructure (SharedArrayBuffer, messaging, lifecycle)
- [ ] Math library (Vec3, Quat, Mat3, SIMD-optimized)
- [ ] Basic physics worker (rigid body, RK4, motor model)
- [ ] Basic FC worker (sensor sim, complementary filter, PID, mixer)
- [ ] Input system (Gamepad API, calibration, channel mapping)
- [ ] Minimal renderer (WebGPU triangle, camera, drone model)

### Phase 2: Flight Core (Weeks 3–4)
- [ ] Complete aerodynamics (drag, ground effect, wind)
- [ ] Betaflight PID parity (all features: D-filter, anti-gravity, RPM filter)
- [ ] Flight modes (ACRO, ANGLE, HORIZON, FAILSAFE)
- [ ] CLI parser (Betaflight command subset)
- [ ] Transmitter support (Web Serial, CRSF parsing)
- [ ] FPV camera (physical model, distortion, latency)
- [ ] Basic terrain (procedural heightfield, collision)

### Phase 3: Graphics & Environment (Weeks 5–7)
- [ ] Deferred renderer (G-buffer, clustered lighting)
- [ ] Atmosphere (Bruneton sky, volumetric clouds)
- [ ] Vegetation (grass compute shader, instanced trees)
- [ ] Water (screen-space reflections, refraction)
- [ ] Drone rendering (PBR, damage, LED strips, prop discs)
- [ ] FPV post-process stack (all effects physics-driven)
- [ ] Graphics presets + auto-detect

### Phase 4: Training & Telemetry (Weeks 8–10)
- [ ] Maneuver definition format + validation
- [ ] Maneuver engine (phase detection, scoring)
- [ ] Course engine (gates, timing, ghost replay)
- [ ] Progression system (profile, stats, achievements)
- [ ] Ghost system (record, compress, playback, compare)
- [ ] Telemetry recorder (binary format, multi-stream)
- [ ] Exporters (CSV, KML, Blackbox, JSON)
- [ ] Analysis tools (3D viewer, PID analyzer, motor analyzer)

### Phase 5: Multiplayer & Polish (Weeks 11–13)
- [ ] WebRTC signaling server (Node.js, minimal)
- [ ] Rollback netcode (deterministic sim, input sync)
- [ ] Session types (race, freestyle, training, spectator)
- [ ] Leaderboards (cloud backend)
- [ ] Cloud sync (profile, logs, ghosts)
- [ ] PWA (offline, installable, background sync)
- [ ] Accessibility (color blind, reduced motion, screen reader)
- [ ] Documentation (user guide, developer guide, API reference)

### Phase 6: Launch & Iterate (Weeks 14+)
- [ ] Beta testing program
- [ ] Performance optimization (profiling-guided)
- [ ] Content: more tracks, quads, maneuvers
- [ ] Community features (replay sharing, liveries)
- [ ] Mobile/touch optimization
- [ ] WebXR support (VR FPV)

---

## 15. Risk Assessment & Mitigation

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|------------|
| **WebGPU unavailable** | Medium | High | WebGL 2 fallback from day 1; feature detection |
| **SharedArrayBuffer requires COOP/COEP** | High | Medium | Configure headers; fallback to postMessage (slower) |
| **Floating-point determinism across browsers** | Medium | High | Fixed-point math for netcode; test matrix |
| **Physics performance at 1 kHz** | Medium | High | WASM compilation for hot paths; profiling |
| **Audio latency (Web Audio)** | Low | Medium | AudioWorklet for synthesis; buffer tuning |
| **Web Serial API permissions UX** | Medium | Medium | Clear instructions; fallback to Gamepad API |
| **IndexedDB quota limits** | Low | Medium | Quota monitoring; cloud offload option |
| **Mobile performance** | High | High | Separate quality preset; touch-first UI |
| **Scope creep** | High | High | Phase gates; frozen requirements per phase |

---

## 16. Success Criteria (Definition of Done)

### Engineering
- [ ] Physics deterministic at 1 kHz across Chrome/Firefox/Safari/Edge
- [ ] FC PID output matches Betaflight 4.5 reference logs (±2%)
- [ ] Input latency < 20 ms stick-to-screen (measured)
- [ ] 60 FPS sustained on 5-year-old hardware (GTX 1060 / RX 580 class)
- [ ] Zero memory leaks over 1-hour session
- [ ] All TypeScript strict mode, zero `any` in core

### Product
- [ ] Complete maneuver library (20+ maneuvers)
- [ ] 5+ training courses with ghosts
- [ ] Multiplayer race with 8 pilots, < 3 frame rollback typical
- [ ] Profile persistence across sessions/devices
- [ ] Export compatible with Betaflight Configurator
- [ ] PWA installable, works offline

### Visual
- [ ] FPV camera indistinguishable from real footage (A/B test)
- [ ] Environment runs at 60 FPS Ultra on RTX 3060
- [ ] No "gamey" artifacts — all effects physics-derived

---

## 17. Immediate Next Steps

1. **Initialize repository** with TypeScript, Vite, Vitest, ESLint, Prettier
2. **Configure CI/CD** (GitHub Actions: lint, type-check, test, build)
3. **Implement worker infrastructure** (SharedArrayBuffer, message protocol, lifecycle)
4. **Build math library** with benchmarks
5. **Prototype physics worker** (rigid body + motor + RK4) with visual debug overlay
6. **Prototype FC worker** (sensor sim + PID + mixer) with Betaflight CLI
7. **Create first drone GLTF model** (procedural or Blender → GLTF)
8. **Establish rendering baseline** (WebGPU triangle → textured quad → drone)

---

## 18. Handoff Notes

**For Elena Rostova (Frontend/UX):** UI/UX layer in `src/ui/` needs design system. FPV HUD/OSD must match Betaflight exactly. Calibration wizard UX critical for transmitter onboarding.

**For Tariq Chen (DevOps/Backend):** Cloud sync API (PHP/MySQL on Hostinger) needed for Phase 5. WebRTC signaling server (Node.js) needed for multiplayer. CDN for assets.

**For Gwen Montgomery (QA):** Test matrix: browsers × OS × hardware × input devices. Determinism tests for physics/FC. Network condition simulation for multiplayer.

**For Stella Sage (Editorial):** Article series planned: "Building a Browser-Based FPV Simulator" (8 parts: Physics, FC, Graphics, Input, Training, Telemetry, Multiplayer, Launch).

---

*End of Architectural Design Document*  
*Marcus Vance — Lead Systems & Backend Architect*  
*2026-11-23*