import { defineConfig } from "vite";
import preact from "@preact/preset-vite";

// `npm run dev` proxies the API to a running dwar (DWAR=http://robot:8080).
const target = process.env.DWAR ?? "http://127.0.0.1:8080";

export default defineConfig({
  plugins: [preact()],
  build: { outDir: "dist", emptyOutDir: true, target: "es2020" },
  server: { proxy: { "/api": { target, ws: true } } },
});
