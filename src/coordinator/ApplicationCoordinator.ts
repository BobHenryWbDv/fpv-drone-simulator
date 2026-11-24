/**
 * Application Coordinator - Main thread state machine and routing
 * Coordinates workers, rendering, input, UI
 */

import { Vec3, Quat } from '@core/math';

// ============================================================================
// Types
// ============================================================================

type AppState = 
  | 'INITIALIZING'
  | 'LOADING_ASSETS'
  | 'READY'
  | 'FLYING'
  | 'PAUSED'
  | 'SETTINGS'
  | 'CALIBRATION'
  | 'TRAINING'
  | 'MULTIPLAYER_LOBBY'
  | 'MULTIPLAYER_FLYING'
  | 'REPLAY'
  | 'ERROR';

interface WorkerMessage {
  type: string;
  [key: string]: any;
}

interface PhysicsState {
  position: Vec3;
  velocity: Vec3;
  orientation: Quat;
  angularVelocity: Vec3;
  motorRPM: number[];
  motorThrust: number[];
  forces: Vec3;
  torques: Vec3;
  acceleration: Vec3;
  angularAcceleration: Vec3;
  wind: Vec3;
  batteryVoltage: number;
  batteryCurrent: number;
  simTime: number;
  frameCount: number;
  flags: number;
}

// ============================================================================
// Shared Buffer Manager
// ============================================================================

class SharedBufferManager {
  private physicsBuffer: SharedArrayBuffer;
  private controlBuffer: SharedArrayBuffer;
  private physicsState: Float32Array;
  private controlInput: Float32Array;
  
  constructor() {
    // Physics state: 64 floats
    this.physicsBuffer = new SharedArrayBuffer(64 * 4);
    this.physicsState = new Float32Array(this.physicsBuffer);
    
    // Control input: 16 floats
    this.controlBuffer = new SharedArrayBuffer(16 * 4);
    this.controlInput = new Float32Array(this.controlBuffer);
  }
  
  getPhysicsBuffer(): SharedArrayBuffer {
    return this.physicsBuffer;
  }
  
  getControlBuffer(): SharedArrayBuffer {
    return this.controlBuffer;
  }
  
  getPhysicsState(): Float32Array {
    return this.physicsState;
  }
  
  getControlInput(): Float32Array {
    return this.controlInput;
  }
  
  readPhysicsState(): PhysicsState {
    const ps = this.physicsState;
    return {
      position: new Float32Array([ps[0], ps[1], ps[2]]) as Vec3,
      velocity: new Float32Array([ps[3], ps[4], ps[5]]) as Vec3,
      orientation: new Float32Array([ps[6], ps[7], ps[8], ps[9]]) as Quat,
      angularVelocity: new Float32Array([ps[10], ps[11], ps[12]]) as Vec3,
      motorRPM: [
        ps[13], ps[14], ps[15], ps[16]
      ],
      motorThrust: [
        ps[17], ps[18], ps[19], ps[20]
      ],
      forces: new Float32Array([ps[25], ps[26], ps[27]]) as Vec3,
      torques: new Float32Array([ps[28], ps[29], ps[30]]) as Vec3,
      acceleration: new Float32Array([ps[31], ps[32], ps[33]]) as Vec3,
      angularAcceleration: new Float32Array([ps[34], ps[35], ps[36]]) as Vec3,
      wind: new Float32Array([ps[37], ps[38], ps[39]]) as Vec3,
      batteryVoltage: ps[40],
      batteryCurrent: ps[41],
      simTime: ps[42],
      frameCount: ps[43],
      flags: ps[44],
    };
  }
}

// ============================================================================
// Worker Manager
// ============================================================================

class WorkerManager {
  private workers: Map<string, Worker> = new Map();
  private messageHandlers: Map<string, (data: any) => void> = new Map();
  private sharedBuffers: SharedBufferManager;
  
  constructor(sharedBuffers: SharedBufferManager) {
    this.sharedBuffers = sharedBuffers;
  }
  
  async spawnWorker(name: string, scriptUrl: string, config: any = {}): Promise<Worker> {
    const worker = new Worker(scriptUrl, { type: 'module', name });
    
    worker.onmessage = (event) => {
      const message = event.data;
      const handler = this.messageHandlers.get(`${name}:${message.type}`);
      if (handler) {
        handler(message);
      } else {
        // Default handler
        this.handleDefaultMessage(name, message);
      }
    };
    
    worker.onerror = (error) => {
      console.error(`[WorkerManager] Worker ${name} error:`, error);
    };
    
    // Initialize worker
    await this.sendMessage(name, {
      type: 'init',
      config,
      physicsBuffer: this.sharedBuffers.getPhysicsBuffer(),
      controlBuffer: this.sharedBuffers.getControlBuffer(),
    });
    
    // Wait for ready
    await new Promise<void>((resolve) => {
      const handler = (msg: any) => {
        if (msg.type === 'ready') {
          this.messageHandlers.delete(`${name}:ready`);
          resolve();
        }
      };
      this.messageHandlers.set(`${name}:ready`, handler);
    });
    
    this.workers.set(name, worker);
    console.log(`[WorkerManager] Spawned worker: ${name}`);
    
    return worker;
  }
  
  async sendMessage(name: string, message: WorkerMessage): Promise<void> {
    const worker = this.workers.get(name);
    if (!worker) {
      throw new Error(`Worker ${name} not found`);
    }
    worker.postMessage(message);
  }
  
  onMessage(name: string, type: string, handler: (data: any) => void): void {
    this.messageHandlers.set(`${name}:${type}`, handler);
  }
  
  private handleDefaultMessage(name: string, message: WorkerMessage): void {
    // Forward to any global handlers
    const globalHandler = this.messageHandlers.get(`*:${message.type}`);
    if (globalHandler) {
      globalHandler({ worker: name, ...message });
    }
  }
  
  terminate(name: string): void {
    const worker = this.workers.get(name);
    if (worker) {
      worker.terminate();
      this.workers.delete(name);
    }
  }
  
  terminateAll(): void {
    for (const [name, worker] of this.workers) {
      worker.terminate();
    }
    this.workers.clear();
  }
  
  getWorker(name: string): Worker | undefined {
    return this.workers.get(name);
  }
}

// ============================================================================
// Application Coordinator
// ============================================================================

export class ApplicationCoordinator {
  private state: AppState = 'INITIALIZING';
  private sharedBuffers: SharedBufferManager;
  private workerManager: WorkerManager;
  private canvas: HTMLCanvasElement | null = null;
  private renderingContext: GPUCanvasContext | WebGL2RenderingContext | null = null;
  private animationFrameId: number | null = null;
  private lastFrameTime = 0;
  private frameCount = 0;
  private fps = 0;
  private fpsUpdateTime = 0;
  
  // Callbacks
  private onStateChange: ((state: AppState) => void)[] = [];
  private onPhysicsUpdate: ((state: PhysicsState) => void)[] = [];
  private onTelemetry: ((data: any) => void)[] = [];
  
  constructor() {
    this.sharedBuffers = new SharedBufferManager();
    this.workerManager = new WorkerManager(this.sharedBuffers);
  }
  
  async initialize(): Promise<void> {
    console.log('[Coordinator] Initializing...');
    
    // Create canvas
    this.createCanvas();
    
    // Check for WebGPU
    const hasWebGPU = await this.initWebGPU();
    if (!hasWebGPU) {
      console.warn('[Coordinator] WebGPU not available, falling back to WebGL 2');
      await this.initWebGL2();
    }
    
    // Spawn workers
    await this.spawnWorkers();
    
    // Set up physics state listener
    this.workerManager.onMessage('physics', 'state', (msg) => {
      if (msg.physicsState) {
        this.onPhysicsStateUpdate(msg.physicsState);
      }
    });
    
    this.workerManager.onMessage('fc', 'telemetry', (msg) => {
      if (msg.data) {
        this.onTelemetryUpdate(msg.data);
      }
    });
    
    this.workerManager.onMessage('physics', 'perf', (msg) => {
      console.log(`[Physics] Step: ${msg.stepTime.toFixed(3)}ms, Avg: ${msg.avgStepTime.toFixed(3)}ms`);
    });
    
    this.workerManager.onMessage('physics', 'event', (msg) => {
      console.log(`[Physics Event] ${msg.event}:`, msg.data);
      if (msg.event === 'crash') {
        this.transitionTo('READY'); // Reset to ready after crash
      }
    });
    
    // Input handling
    this.setupInput();
    
    // Transition to ready
    this.transitionTo('READY');
    
    console.log('[Coordinator] Initialization complete');
  }
  
  private createCanvas(): void {
    this.canvas = document.createElement('canvas');
    this.canvas.id = 'sim-canvas';
    this.canvas.style.width = '100%';
    this.canvas.style.height = '100%';
    this.canvas.style.display = 'block';
    this.canvas.style.background = '#000';
    
    const container = document.getElementById('app') || document.body;
    container.appendChild(this.canvas);
    
    // Handle resize
    const resize = () => {
      const rect = this.canvas!.getBoundingClientRect();
      this.canvas!.width = rect.width * devicePixelRatio;
      this.canvas!.height = rect.height * devicePixelRatio;
    };
    
    resize();
    window.addEventListener('resize', resize);
  }
  
  private async initWebGPU(): Promise<boolean> {
    if (!navigator.gpu) return false;
    
    try {
      const adapter = await navigator.gpu.requestAdapter({
        powerPreference: 'high-performance',
      });
      
      if (!adapter) return false;
      
      const device = await adapter.requestDevice({
        requiredFeatures: [
          'depth-clip-control',
          'texture-compression-bc',
          'texture-compression-etc2',
          'texture-compression-astc',
          'timestamp-query',
          'subgroups',
        ],
        requiredLimits: {
          maxTextureDimension2D: 8192,
          maxStorageBufferBindingSize: 256 * 1024 * 1024,
        },
      });
      
      const context = this.canvas!.getContext('webgpu');
      if (!context) return false;
      
      const format = navigator.gpu.getPreferredCanvasFormat();
      context.configure({
        device,
        format,
        alphaMode: 'opaque',
        usage: GPUTextureUsage.RENDER_ATTACHMENT | GPUTextureUsage.COPY_SRC,
      });
      
      this.renderingContext = context;
      (this as any).gpuDevice = device;
      
      console.log('[Coordinator] WebGPU initialized:', adapter.info);
      return true;
      
    } catch (error) {
      console.warn('[Coordinator] WebGPU init failed:', error);
      return false;
    }
  }
  
  private async initWebGL2(): Promise<void> {
    const gl = this.canvas!.getContext('webgl2', {
      alpha: false,
      antialias: true,
      depth: true,
      stencil: true,
      preserveDrawingBuffer: false,
      powerPreference: 'high-performance',
    });
    
    if (!gl) {
      throw new Error('WebGL 2 not available');
    }
    
    this.renderingContext = gl;
    console.log('[Coordinator] WebGL 2 initialized');
  }
  
  private async spawnWorkers(): Promise<void> {
    // Import worker scripts as URLs
    const physicsWorkerUrl = new URL('../workers/physics.worker.ts', import.meta.url).href;
    const fcWorkerUrl = new URL('../workers/fc.worker.ts', import.meta.url).href;
    
    await this.workerManager.spawnWorker('physics', physicsWorkerUrl, {
      mass: 0.580,
      loopRate: 1000,
    });
    
    await this.workerManager.spawnWorker('fc', fcWorkerUrl, {
      loopRate: 2000,
    });
    
    // TODO: Spawn training, telemetry, network workers
  }
  
  private setupInput(): void {
    // Keyboard input for testing
    window.addEventListener('keydown', (e) => this.handleKeyDown(e));
    window.addEventListener('keyup', (e) => this.handleKeyUp(e));
    
    // Gamepad API polling
    this.startGamepadPolling();
  }
  
  private handleKeyDown(e: KeyboardEvent): void {
    // Simple keyboard control for testing
    const ci = this.sharedBuffers.getControlInput();
    
    switch (e.code) {
      case 'KeyW': ci[4] = 1; break; // Throttle
      case 'KeyS': ci[4] = 0; break;
      case 'ArrowUp': ci[5] = 10; break; // Pitch rate
      case 'ArrowDown': ci[5] = -10; break;
      case 'ArrowLeft': ci[6] = -10; break; // Roll rate
      case 'ArrowRight': ci[6] = 10; break;
      case 'KeyA': ci[7] = -10; break; // Yaw rate
      case 'KeyD': ci[7] = 10; break;
      case 'Space': ci[12] = 1; break; // Arm
      case 'Escape': this.togglePause(); break;
    }
  }
  
  private handleKeyUp(e: KeyboardEvent): void {
    const ci = this.sharedBuffers.getControlInput();
    
    switch (e.code) {
      case 'KeyW':
      case 'KeyS':
        ci[4] = 0; break;
      case 'ArrowUp':
      case 'ArrowDown':
        ci[5] = 0; break;
      case 'ArrowLeft':
      case 'ArrowRight':
        ci[6] = 0; break;
      case 'KeyA':
      case 'KeyD':
        ci[7] = 0; break;
    }
  }
  
  private startGamepadPolling(): void {
    const poll = () => {
      const gamepads = navigator.getGamepads();
      for (const gp of gamepads) {
        if (gp) {
          this.processGamepad(gp);
        }
      }
      requestAnimationFrame(poll);
    };
    requestAnimationFrame(poll);
  }
  
  private processGamepad(gp: Gamepad): void {
    const ci = this.sharedBuffers.getControlInput();
    
    // Standard gamepad mapping (Mode 2)
    // Left stick: Pitch (up/down) / Roll (left/right)
    // Right stick: Throttle (up/down) / Yaw (left/right)
    
    const axes = gp.axes;
    if (axes.length >= 4) {
      // Roll (left stick X)
      ci[6] = axes[0] * 500 * Math.PI / 180; // deg/s to rad/s
      
      // Pitch (left stick Y, inverted)
      ci[5] = -axes[1] * 500 * Math.PI / 180;
      
      // Throttle (right stick Y, inverted and mapped 0-1)
      ci[4] = Math.max(0, -axes[3]);
      
      // Yaw (right stick X)
      ci[7] = axes[2] * 500 * Math.PI / 180;
    }
    
    // Buttons
    const buttons = gp.buttons;
    if (buttons.length > 0) {
      // Button 0 (A) = Arm
      if (buttons[0].pressed) ci[12] = 1;
    }
    if (buttons.length > 1) {
      // Button 1 (B) = Disarm
      if (buttons[1].pressed) ci[12] = 0;
    }
  }
  
  private onPhysicsStateUpdate(physicsState: Float32Array): void {
    const state = this.sharedBuffers.readPhysicsState();
    
    for (const callback of this.onPhysicsUpdate) {
      try {
        callback(state);
      } catch (error) {
        console.error('[Coordinator] Physics update callback error:', error);
      }
    }
  }
  
  private onTelemetryUpdate(data: any): void {
    for (const callback of this.onTelemetry) {
      try {
        callback(data);
      } catch (error) {
        console.error('[Coordinator] Telemetry callback error:', error);
      }
    }
  }
  
  start(): void {
    if (this.animationFrameId) return;
    
    this.lastFrameTime = performance.now();
    this.animationFrameId = requestAnimationFrame(this.renderLoop.bind(this));
    console.log('[Coordinator] Render loop started');
  }
  
  stop(): void {
    if (this.animationFrameId) {
      cancelAnimationFrame(this.animationFrameId);
      this.animationFrameId = null;
    }
  }
  
  private renderLoop(timestamp: number): void {
    this.animationFrameId = requestAnimationFrame(this.renderLoop.bind(this));
    
    const dt = (timestamp - this.lastFrameTime) / 1000;
    this.lastFrameTime = timestamp;
    
    // Update FPS counter
    this.frameCount++;
    if (timestamp - this.fpsUpdateTime >= 1000) {
      this.fps = this.frameCount;
      this.frameCount = 0;
      this.fpsUpdateTime = timestamp;
    }
    
    // Render frame
    this.renderFrame(dt);
    
    // Update UI
    this.updateHUD();
  }
  
  private renderFrame(dt: number): void {
    // TODO: Actual rendering
    // For now, just clear the canvas
    if (this.renderingContext instanceof GPUCanvasContext) {
      // WebGPU rendering would go here
    } else if (this.renderingContext instanceof WebGL2RenderingContext) {
      const gl = this.renderingContext;
      gl.viewport(0, 0, gl.drawingBufferWidth, gl.drawingBufferHeight);
      gl.clearColor(0.05, 0.1, 0.15, 1.0);
      gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
    }
  }
  
  private updateHUD(): void {
    // Update debug HUD
    const hud = document.getElementById('debug-hud');
    if (hud) {
      const state = this.sharedBuffers.readPhysicsState();
      hud.textContent = `
FPS: ${this.fps}
State: ${this.state}
Pos: [${state.position[0].toFixed(2)}, ${state.position[1].toFixed(2)}, ${state.position[2].toFixed(2)}]
Vel: [${state.velocity[0].toFixed(2)}, ${state.velocity[1].toFixed(2)}, ${state.velocity[2].toFixed(2)}]
RPM: [${state.motorRPM.map(r => r.toFixed(0)).join(', ')}]
Battery: ${state.batteryVoltage.toFixed(2)}V
Sim Time: ${state.simTime.toFixed(2)}s
Flags: ${state.flags.toString(2).padStart(8, '0')}
      `.trim();
    }
  }
  
  transitionTo(newState: AppState): void {
    if (this.state === newState) return;
    
    const oldState = this.state;
    this.state = newState;
    
    console.log(`[Coordinator] State: ${oldState} -> ${newState}`);
    
    for (const callback of this.onStateChange) {
      try {
        callback(newState);
      } catch (error) {
        console.error('[Coordinator] State change callback error:', error);
      }
    }
    
    // Handle state-specific logic
    switch (newState) {
      case 'FLYING':
        this.workerManager.sendMessage('physics', { type: 'resume' });
        this.workerManager.sendMessage('fc', { type: 'resume' });
        break;
      case 'PAUSED':
        this.workerManager.sendMessage('physics', { type: 'pause' });
        this.workerManager.sendMessage('fc', { type: 'pause' });
        break;
      case 'READY':
        this.workerManager.sendMessage('physics', { type: 'reset' });
        this.workerManager.sendMessage('fc', { type: 'reset' });
        break;
    }
  }
  
  getState(): AppState {
    return this.state;
  }
  
  onStateChangeCallback(cb: (state: AppState) => void): () => void {
    this.onStateChange.push(cb);
    return () => {
      const idx = this.onStateChange.indexOf(cb);
      if (idx >= 0) this.onStateChange.splice(idx, 1);
    };
  }
  
  onPhysicsUpdateCallback(cb: (state: PhysicsState) => void): () => void {
    this.onPhysicsUpdate.push(cb);
    return () => {
      const idx = this.onPhysicsUpdate.indexOf(cb);
      if (idx >= 0) this.onPhysicsUpdate.splice(idx, 1);
    };
  }
  
  onTelemetryCallback(cb: (data: any) => void): () => void {
    this.onTelemetry.push(cb);
    return () => {
      const idx = this.onTelemetry.indexOf(cb);
      if (idx >= 0) this.onTelemetry.splice(idx, 1);
    };
  }
  
  // CLI interface for FC
  async sendCLICommand(command: string): Promise<{ success: boolean; response: string }> {
    return new Promise((resolve) => {
      const handler = (msg: any) => {
        if (msg.type === 'cliResponse') {
          this.workerManager.onMessage('fc', 'cliResponse', () => {}); // Remove handler
          resolve({ success: msg.success, response: msg.response });
        }
      };
      this.workerManager.onMessage('fc', 'cliResponse', handler);
      this.workerManager.sendMessage('fc', { type: 'cli', command });
    });
  }
  
  // RC Input
  setRCChannels(channels: number[]): void {
    this.workerManager.sendMessage('fc', { type: 'rcInput', channels });
  }
  
  // Cleanup
  destroy(): void {
    this.stop();
    this.workerManager.terminateAll();
    
    if (this.canvas && this.canvas.parentNode) {
      this.canvas.parentNode.removeChild(this.canvas);
    }
  }
}

export default ApplicationCoordinator;