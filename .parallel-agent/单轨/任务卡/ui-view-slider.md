---
kind: 任务卡
task_id: ui-view-slider
mode: L3
---

# 任务卡：前端视野滑动条（10~100）

## 目标

- 顶栏新增「视野」滑块：范围 10~100，显示当前值，改变即生效。
- 运行时修改求解器视野：
  - `Solver.set_view_size(v)`：更新 `view_size`，重置增量观测锚点并做一次完整视野观测。
  - `Api.set_view_size(v)`：限制 10~100，更新 `cfg.view_size` 与求解器。
- 中栏实时视野使用 `get_state().view_size` 动态重绘（不再固定 25）。
- 重开后沿用当前视野值（`cfg.view_size` 保留）。

## 非目标

- 不把视野写入 config.toml 持久化。
- 不改求解策略、迷宫生成、配色。
- 不改速度滑块与暂停逻辑。

## 验收

- [ ] 前端出现「视野」滑块，min=10、max=100；拖动时数字与中栏视野尺寸同步变化。
- [ ] `solver.view_size` 随滑块改变；中栏画布边长 = view_size × 16（超出卡片时按 CSS 缩放）。
- [ ] 视野变化后感知仍正确：新视野内墙/通道正确并入已知地图，不产生错误标记。
- [ ] 重开保留当前视野值；`get_state` 含 `view_size`。
- [ ] `pnpm build`、`py_compile` 通过；浏览器无 JS 报错。

## 产物路径

- `src/maze/solver.py`（`set_view_size`）
- `src/maze/visual.py`（`Api.set_view_size`、`get_state` 增加 `view_size`）
- `frontend/src/App.vue`（视野滑块、动态中栏）
- `frontend/src/style.css`（滑块样式如需）
- `config.toml` 不变

## 红线

- 不修改 `.venv/`、不全局安装依赖。
- 不把 `.parallel-agent/` 元文件混入业务提交。
- 没有命令与输出证据，不宣称完成。
