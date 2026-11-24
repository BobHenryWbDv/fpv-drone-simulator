# FPV Drone Simulator

A comprehensive browser-based FPV drone simulator and pilot-training platform with Betaflight-compatible flight controller, realistic quadcopter physics, real transmitter/gamepad compatibility, high-quality 3D graphics, realistic FPV camera presentation, structured pilot training, flight telemetry/recording/analysis, racing/freestyle/multiplayer capabilities, and persistent pilot profiles.

## Features

- **Betaflight-Compatible Flight Controller**: PID loops, mixer, flight modes (ACRO, ANGLE, HORIZON), failsafe, RPM filter, dynamic LPF
- **Realistic Quadcopter Physics**: Rigid-body dynamics (RK4 integration), BLDC motor model, aerodynamics, ground effect, wind simulation
- **Real Transmitter Support**: USB HID (FrSky, RadioMaster, Jumper, FlySky), Web Serial API (Crossfire/ELRS CRSF), gamepad support
- **High-Quality 3D Graphics**: WebGPU primary with WebGL 2 fallback, deferred rendering, volumetric clouds, procedural terrain
- **Realistic FPV Camera**: Physical camera model (FOV, distortion, vignetting), rolling shutter, motor vibration, latency simulation
- **Structured Training**: Maneuver library (Split-S, Power Loop, Cuban 8, etc.), courses, ghost replay, progression system
- **Telemetry & Recording**: Multi-rate logging, Blackbox-compatible export, PID tuning analyzer, 3D trajectory viewer
- **Multiplayer**: WebRTC DataChannels, rollback netcode (GGPO-style), race/freestyle/training sessions
- **Persistent Profiles**: IndexedDB local storage with optional cloud sync

## Architecture

```
Main Thread (60-144 Hz)          Workers (Fixed Timestep)
┌─────────────────────────┐      ┌─────────────────────────┐
│ Rendering (WebGPU)      │      │ Physics (1 kHz)         │
│ UI/UX                   │      │ Flight Controller (2-4 kHz)│
│ Input Polling           │◄────►│ Training (async)        │
│ Audio                   │      │ Telemetry (async)       │
│ Coordinator             │      │ Network (async)         │
└─────────────────────────┘      └─────────────────────────┘
              ▲                           ▲
              │      SharedArrayBuffer    │
              └───────────────────────────┘
              (Zero-copy state sharing)
```

## Getting Started

### Prerequisites

- Node.js 20+
- Modern browser with WebGPU support (Chrome 113+, Firefox 137+, Safari 16.4+)
- HTTPS or localhost (required for WebGPU, SharedArrayBuffer, Web Serial API)

### Installation

```bash
# Clone repository
git clone https://github.com/BobHenryWbDv/fpv-drone-simulator.git
cd fpv-drone-simulator

# Install dependencies
npm install

# Development server
npm run dev

# Production build
npm run build

# Preview production build
npm run preview

# Run tests
npm run test

# Type checking
npm run typecheck

# Linting
npm run lint

# Formatting
npm run format
```

### Required HTTP Headers

For SharedArrayBuffer support, your server must send:
```
Cross-Origin-Opener-Policy: same-origin
Cross-Origin-Embedder-Policy: require-corp
```

The Vite dev server configures these automatically. For production, configure your web server accordingly.

## Project Structure

```
fpv-drone-simulator/
├── public/                 # Static assets
│   ├── index.html         # Entry point
│   ├── manifest.json      # PWA manifest
│   ├── sw.js              # Service worker
│   ├── assets/            # Models, textures, shaders, audio
│   └── wasm/              # Compiled WASM modules
├── src/                   # Main thread source
│   ├── main.ts            # Application bootstrap
│   ├── coordinator/       # Application state machine
│   ├── core/              # Core systems
│   │   ├── math/          # Vec3, Quat, Mat3, Mat4, Random
│   │   ├── config/        # Quad, Rate, PID profiles
│   │   └── physics/       # Physics types (shared with worker)
│   ├── graphics/          # Rendering engine
│   ├── input/             # Input handling
│   ├── training/          # Training system
│   ├── telemetry/         # Telemetry & recording
│   ├── multiplayer/       # Networking
│   ├── persistence/       # IndexedDB & cloud sync
│   ├── ui/                # UI components
│   ├── audio/             # Audio engine
│   └── utils/             # Utilities
├── workers/               # Web Workers
│   ├── physics.worker.ts  # Physics simulation (1 kHz)
│   ├── fc.worker.ts       # Flight controller (2-4 kHz)
│   ├── training.worker.ts # Maneuver detection
│   ├── telemetry.worker.ts# Data logging
│   └── network.worker.ts  # WebRTC networking
├── docs/                  # Documentation
├── tests/                 # Test files
├── package.json
├── tsconfig.json
├── vite.config.ts
└── vite.worker.config.ts
```

## Technology Stack

| Layer | Technology |
|-------|------------|
| Language | TypeScript 5.6+ |
| Build | Vite 5.x |
| Graphics | WebGPU (WebGL 2 fallback) |
| Math | gl-matrix + custom SIMD-optimized |
| Workers | Web Workers + SharedArrayBuffer |
| Storage | IndexedDB (idb) |
| Networking | WebRTC DataChannels |
| Testing | Vitest + Playwright |
| Linting | ESLint + TypeScript ESLint |
| Formatting | Prettier |

## Performance Targets

| Metric | Target |
|--------|--------|
| Physics Timestep | 1 ms (1 kHz) fixed |
| FC Loop Rate | 2-4 kHz configurable |
| Frame Rate | 60-144 FPS (VRR) |
| Frame Time (Main) | < 8 ms (120 FPS) |
| Frame Time (Graphics) | < 6 ms (Ultra) |
| Memory (Total) | < 512 MB |
| Cold Load (cached) | < 3 s |
| Input Latency | < 20 ms |

## Browser Support

| Browser | Version | WebGPU | SharedArrayBuffer | Web Serial |
|---------|---------|--------|-------------------|------------|
| Chrome | 113+ | ✅ | ✅ | ✅ |
| Edge | 113+ | ✅ | ✅ | ✅ |
| Firefox | 137+ | ✅ | ✅ | ❌ |
| Safari | 16.4+ | ✅ | ✅ | ❌ |

*Web Serial API requires HTTPS and user gesture. Firefox/Safari users can use Gamepad API for transmitter input.*

## Development Phases

- **Phase 1** (Weeks 1-2): Foundation, workers, math, basic physics/FC
- **Phase 2** (Weeks 3-4): Flight core, Betaflight PID parity, transmitter support
- **Phase 3** (Weeks 5-7): Graphics, environment, FPV camera
- **Phase 4** (Weeks 8-10): Training, telemetry, analysis tools
- **Phase 5** (Weeks 11-13): Multiplayer, cloud sync, PWA
- **Phase 6** (Weeks 14+): Beta, optimization, content, WebXR

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make changes with tests
4. Run `npm run lint` and `npm run typecheck`
5. Submit a pull request

## License

MIT License - see [LICENSE](LICENSE) for details.

## Author

Marcus Vance (Lead Systems & Backend Architect)  
marcus.vance@rexbondan.com

## Acknowledgments

- Betaflight developers for the open-source flight controller reference
- The FPV community for maneuver definitions and training methodology
- WebGPU Working Group for the modern graphics API