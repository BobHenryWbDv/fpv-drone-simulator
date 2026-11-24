import { defineConfig } from 'vite';
import { VitePWA } from 'vite-plugin-pwa';
import path from 'path';

export default defineConfig({
  root: '.',
  base: './',
  publicDir: 'public',
  build: {
    outDir: 'dist',
    assetsDir: 'assets',
    sourcemap: true,
    minify: 'esbuild',
    target: 'es2022',
    cssCodeSplit: true,
    rollupOptions: {
      input: {
        main: path.resolve(__dirname, 'public/index.html'),
      },
      output: {
        manualChunks: {
          'vendor-math': ['gl-matrix'],
          'vendor-storage': ['idb'],
          'vendor-compression': ['zstd-codec'],
        },
      },
    },
    worker: {
      format: 'es',
      plugins: () => [],
    },
  },
  server: {
    port: 5173,
    host: true,
    headers: {
      'Cross-Origin-Opener-Policy': 'same-origin',
      'Cross-Origin-Embedder-Policy': 'require-corp',
    },
  },
  preview: {
    port: 4173,
    host: true,
    headers: {
      'Cross-Origin-Opener-Policy': 'same-origin',
      'Cross-Origin-Embedder-Policy': 'require-corp',
    },
  },
  resolve: {
    alias: {
      '@': path.resolve(__dirname, 'src'),
      '@core': path.resolve(__dirname, 'src/core'),
      '@graphics': path.resolve(__dirname, 'src/graphics'),
      '@input': path.resolve(__dirname, 'src/input'),
      '@training': path.resolve(__dirname, 'src/training'),
      '@telemetry': path.resolve(__dirname, 'src/telemetry'),
      '@multiplayer': path.resolve(__dirname, 'src/multiplayer'),
      '@persistence': path.resolve(__dirname, 'src/persistence'),
      '@ui': path.resolve(__dirname, 'src/ui'),
      '@audio': path.resolve(__dirname, 'src/audio'),
      '@utils': path.resolve(__dirname, 'src/utils'),
      '@workers': path.resolve(__dirname, 'workers'),
    },
  },
  optimizeDeps: {
    include: ['gl-matrix', 'idb', 'zstd-codec'],
    exclude: [],
  },
  plugins: [
    VitePWA({
      registerType: 'autoUpdate',
      includeAssets: ['favicon.ico', 'robots.txt'],
      manifest: {
        name: 'FPV Drone Simulator',
        short_name: 'FPV Sim',
        description: 'Browser-based FPV drone simulator with Betaflight-compatible flight controller',
        theme_color: '#0f172a',
        background_color: '#0f172a',
        display: 'fullscreen',
        orientation: 'landscape-primary',
        scope: '/',
        start_url: '/',
        icons: [
          { src: '/assets/icons/icon-72.png', sizes: '72x72', type: 'image/png' },
          { src: '/assets/icons/icon-96.png', sizes: '96x96', type: 'image/png' },
          { src: '/assets/icons/icon-128.png', sizes: '128x128', type: 'image/png' },
          { src: '/assets/icons/icon-144.png', sizes: '144x144', type: 'image/png' },
          { src: '/assets/icons/icon-152.png', sizes: '152x152', type: 'image/png' },
          { src: '/assets/icons/icon-192.png', sizes: '192x192', type: 'image/png', purpose: 'any maskable' },
          { src: '/assets/icons/icon-384.png', sizes: '384x384', type: 'image/png' },
          { src: '/assets/icons/icon-512.png', sizes: '512x512', type: 'image/png', purpose: 'any maskable' },
        ],
      },
      workbox: {
        globPatterns: ['**/*.{js,css,html,ico,png,svg,woff2,wasm,gltf,bin}'],
        maximumFileSizeToCacheInBytes: 50 * 1024 * 1024,
        cleanupOutdatedCaches: true,
        runtimeCaching: [
          {
            urlPattern: /^https:\/\/fonts\.googleapis\.com\/.*/i,
            handler: 'CacheFirst',
            options: {
              cacheName: 'google-fonts-cache',
              expiration: { maxEntries: 10, maxAgeSeconds: 60 * 60 * 24 * 365 },
              cacheableResponse: { statuses: [0, 200] },
            },
          },
          {
            urlPattern: /^https:\/\/fonts\.gstatic\.com\/.*/i,
            handler: 'CacheFirst',
            options: {
              cacheName: 'gstatic-fonts-cache',
              expiration: { maxEntries: 10, maxAgeSeconds: 60 * 60 * 24 * 365 },
              cacheableResponse: { statuses: [0, 200] },
            },
          },
        ],
      },
      devOptions: {
        enabled: true,
        type: 'module',
      },
    }),
  ],
  esbuild: {
    treeShaking: true,
    pure: ['console.log', 'console.debug'],
    drop: process.env.NODE_ENV === 'production' ? ['console', 'debugger'] : [],
  },
});