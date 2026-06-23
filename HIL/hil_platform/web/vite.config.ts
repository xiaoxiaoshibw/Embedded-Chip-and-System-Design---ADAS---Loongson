import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";

// 开发期把 /api 与 /ws 代理到 FastAPI（默认 8000），避免 CORS，
// 同时让前端用相对路径，构建后可由 FastAPI 同源托管。
const API_TARGET = process.env.HIL_API || "http://127.0.0.1:8000";

export default defineConfig({
  plugins: [react()],
  server: {
    port: 5173,
    proxy: {
      "/api": { target: API_TARGET, changeOrigin: true },
      "/ws": { target: API_TARGET, ws: true, changeOrigin: true },
    },
  },
  build: {
    outDir: "dist",
    chunkSizeWarningLimit: 1500,
  },
});
