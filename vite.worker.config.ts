import { defineConfig } from 'vite';
import path from 'path';

export default defineConfig({
  root: '.',
  base: './',
  build: {
    outDir: 'dist/workers',
    assetsDir: '',
    sourcemap: true,
    minify: 'esbuild',
    target: 'es2022',
    rollupOptions: {
      input: {
        physics: path.resolve(__dirname, 'workers/physics.worker.ts'),
        fc: path.resolve(__dirname, 'workers/fc.worker.ts'),
        training: path.resolve(__dirname, 'workers/training.worker.ts'),
        telemetry: path.resolve(__dirname, 'workers/telemetry.worker.ts'),
        network: path.resolve(__dirname, 'workers/network.worker.ts'),
      },
      output: {
        format: 'es',
        entryFileNames: '[name].worker.js',
        chunkFileNames: '[name].worker.chunk.js',
        assetFileNames: '[name].[ext]',
      },
    },
  },
  resolve: {
    alias: {
      '@': path.resolve(__dirname, 'src'),
      '@core': path.resolve(__dirname, 'src/core'),
      '@utils': path.resolve(__dirname, 'src/utils'),
    },
  },
  esbuild: {
    treeShaking: true,
    pure: ['console.log', 'console.debug'],
    drop: process.env.NODE_ENV === 'production' ? ['console', 'debugger'] : [],
  },
});