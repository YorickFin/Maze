/** 在没有 pywebview 的浏览器环境里安装兼容 Api，使前端可独立运行（GitHub Pages）。 */
import { WebApi } from './api'

interface WebviewWindow extends Window {
  pywebview?: { api?: unknown }
}

export function installWebApi(): void {
  const win = window as WebviewWindow
  if (win.pywebview?.api) return
  // 桌面端由 __main__.py 在 URL 上带 host=pywebview，避免误装浏览器引擎
  const params = new URLSearchParams(window.location.search)
  if (params.get('host') === 'pywebview') return

  const api = new WebApi()
  win.pywebview = { api }
  // App.vue 在拿不到 api 时会监听该事件
  window.dispatchEvent(new Event('pywebviewready'))
}
