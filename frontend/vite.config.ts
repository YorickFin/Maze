import vue from '@vitejs/plugin-vue'
import { defineConfig } from 'vite'

// https://vite.dev/config/
export default defineConfig({
  // 相对路径：本地静态服务与 GitHub Pages 子路径（/Maze/）都能直接使用
  base: './',
  plugins: [vue()],
})
