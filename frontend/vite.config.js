import react from '@vitejs/plugin-react'
import { defineConfig } from 'vite'

// Decision 11 (docs/decisions.md): the dev server proxies /ws to the real axum backend
// (src/dashboard/server.rs) so `npm run dev` talks to a live robot fleet, not a mock —
// `npm run build`'s static output is what axum actually serves in production, this proxy
// only exists for local development.
export default defineConfig({
  plugins: [react()],
  server: {
    proxy: {
      '/history': 'http://127.0.0.1:8080',
      '/api': 'http://127.0.0.1:8080',
      '/ws': {
        target: 'ws://127.0.0.1:8080',
        ws: true,
      },
    },
  },
})
