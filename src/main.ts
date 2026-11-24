/**
 * Main Application Entry Point
 * Bootstrap the FPV Drone Simulator
 */

import { ApplicationCoordinator } from '@coordinator/ApplicationCoordinator';

// Global initialization
declare global {
  interface Window {
    __FPV_SIM__: {
      coordinator: ApplicationCoordinator;
      version: string;
    };
  }
}

// Version from package.json
const VERSION = '0.1.0';

// Initialize when DOM is ready
async function init(): Promise<void> {
  console.log(`[FPV Sim] Starting v${VERSION}...`);
  
  try {
    // Create coordinator
    const coordinator = new ApplicationCoordinator();
    await coordinator.initialize();
    
    // Expose for debugging
    window.__FPV_SIM__ = {
      coordinator,
      version: VERSION,
    };
    
    console.log('[FPV Sim] Initialized successfully');
    
    // Start render loop
    coordinator.start();
    
  } catch (error) {
    console.error('[FPV Sim] Initialization failed:', error);
    showErrorScreen(error instanceof Error ? error.message : 'Unknown error');
  }
}

function showErrorScreen(message: string): void {
  const container = document.getElementById('app') || document.body;
  container.innerHTML = `
    <div style="
      position: fixed; top: 0; left: 0; width: 100%; height: 100%;
      background: #0f172a; color: #e2e8f0;
      display: flex; flex-direction: column; align-items: center; justify-content: center;
      font-family: system-ui, sans-serif; padding: 2rem; text-align: center;
    ">
      <h1 style="font-size: 2rem; margin-bottom: 1rem; color: #ef4444;">FPV Drone Simulator</h1>
      <p style="font-size: 1.25rem; margin-bottom: 2rem; max-width: 600px;">
        Failed to initialize: ${message}
      </p>
      <button onclick="location.reload()" style="
        padding: 0.75rem 1.5rem; font-size: 1rem;
        background: #3b82f6; color: white; border: none; border-radius: 0.5rem;
        cursor: pointer;
      ">Reload</button>
      <details style="margin-top: 2rem; text-align: left; max-width: 600px;">
        <summary style="cursor: pointer; color: #94a3b8;">Technical Details</summary>
        <pre style="margin-top: 1rem; padding: 1rem; background: #1e293b; border-radius: 0.5rem; overflow: auto;">
${message}
        </pre>
      </details>
    </div>
  `;
}

// Check for required APIs
function checkRequirements(): { supported: boolean; missing: string[] } {
  const missing: string[] = [];
  
  if (!window.Worker) missing.push('Web Workers');
  if (!window.SharedArrayBuffer) missing.push('SharedArrayBuffer (requires COOP/COEP headers)');
  if (!navigator.gpu && !window.WebGL2RenderingContext) missing.push('WebGPU or WebGL 2');
  if (!window.RequestAnimationFrame) missing.push('requestAnimationFrame');
  if (!window.OffscreenCanvas) missing.push('OffscreenCanvas');
  
  return {
    supported: missing.length === 0,
    missing,
  };
}

// Polyfill check
const { supported, missing } = checkRequirements();
if (!supported) {
  console.warn('[FPV Sim] Missing required APIs:', missing);
  // Don't block - some features may work with fallbacks
}

// Start initialization
if (document.readyState === 'loading') {
  document.addEventListener('DOMContentLoaded', init);
} else {
  init();
}

// Handle unhandled errors
window.addEventListener('error', (event) => {
  console.error('[FPV Sim] Unhandled error:', event.error);
});

window.addEventListener('unhandledrejection', (event) => {
  console.error('[FPV Sim] Unhandled rejection:', event.reason);
});

export {};