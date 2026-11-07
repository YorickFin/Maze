---
kind: 任务卡
task_id: frontend-vite-migration
mode: L3
---

# 任务卡：前端迁移到 pywebview + Vite（Vue 3）

## 目标

把现有内联 HTML 三栏界面迁移到 `frontend/` 的 Vue 3 + Vite + TS 工程，并保持全部功能：

- 三栏：左=已探索合成图、中=实时视野、右=完整地图；配色与语义不变。
- 顶栏：暂停、策略下拉（聚类 gain / 沿墙 / 前沿规划）、速度、重开、种子/步数/受阻/策略/状态。
- 重开：换随机种子重建地图与求解器，前端重置并重载地图。
- 视觉：深色简洁风格，卡片式三栏、清晰的图例与状态标签。

## 架构

- **前端**（`frontend/src`）：
  - `App.vue` 承载界面与 canvas 绘制逻辑；`style.css` 全局主题。
  - `main.ts` 挂载 App；`index.html` 更新标题与语言。
  - 删除脚手架示例组件与无关资源。
  - 保留增量状态协议：`get_map()` 一次 + `get_state()` 增量。
- **后端**：
  - `__main__.py` 用本地 HTTP 服务（daemon 线程，127.0.0.1 随机端口）托管 `frontend/dist`，pywebview `create_window(url=...)`。
  - 若环境变量 `MAZE_FRONTEND_URL` 存在则直接加载该 URL（便于 `pnpm dev`）。
  - `frontend/dist` 不存在时给出明确构建提示并退出。
  - `visual.py` 仅保留 `Api`，删除内联 HTML 模板与 `build_html`。
- **构建**：`cd frontend && pnpm build` 生成 dist；dist 不入库。

## 非目标

- 不改迷宫生成、求解算法、Api 协议、配置项。
- 不做多页面、登录、国际化、主题切换。
- 不引入新前端依赖（仅用 Vue + Vite 现有依赖）。

## 验收

- [ ] `pnpm build` 在 `frontend/` 通过，生成 `dist/index.html` 与 assets。
- [ ] `uv run maze` 启动后加载 Vue 界面，窗口存活、无控制台报错。
- [ ] 三栏 canvas 正常渲染：右栏黑墙/白通道/灰障碍/蓝起点/红终点；中栏 25×25 视野；左栏已探索区域。
- [ ] 策略下拉 3 项；切换后状态栏策略变化；沿墙/规划行为可观测。
- [ ] 重开：种子变化、地图变化、步数与受阻归零、三栏重绘。
- [ ] 暂停/继续、速度滑块可用。
- [ ] `get_map/get_state/set_strategy/restart` 与现有 Api 行为一致。
- [ ] `py_compile` 通过；浏览器验证（临时 dist + 假 Api 注入）无 JS 报错。

## 产物路径

- `frontend/src/App.vue`、`frontend/src/style.css`、`frontend/src/main.ts`、`frontend/index.html`
- `frontend/vite.config.ts`（如需 base/outDir 调整）
- `src/maze/__main__.py`（本地静态服务 + 加载 dist / dev URL）
- `src/maze/visual.py`（移除内联 HTML，保留 Api）
- `.gitignore`（确认 `frontend/dist` 被忽略）

## 红线

- 不修改 `.venv/`、不全局安装依赖；前端只用 `pnpm`。
- 不把 `.parallel-agent/` 元文件混入业务提交。
- 没有命令与输出证据，不宣称完成。
