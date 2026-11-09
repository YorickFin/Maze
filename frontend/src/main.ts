import { createApp } from 'vue'
import './style.css'
import App from './App.vue'
import { installWebApi } from './engine/install'

// 浏览器环境（如 GitHub Pages）自行安装兼容 Api；pywebview 桌面端会跳过
installWebApi()

createApp(App).mount('#app')
