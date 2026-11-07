---
kind: 任务卡
task_id: ui-strategy-restart
mode: L3
---

# 任务卡：前端策略下拉 + 重开（随机新地图）

## 目标

1. **策略下拉框**（顶栏 `<select>`），三个选项：
   - `cluster`：聚类 gain（`wall_follow=false, cluster_enabled=true`）
   - `wall`：沿墙（`wall_follow=true, cluster_enabled=false`）
   - `frontier`：前沿规划（`wall_follow=false, cluster_enabled=false`）
   - 切换即时生效：清空当前路径、重置模式状态，不重启地图。
2. **重开按钮**：用新的随机种子重新生成迷宫，重建求解器，重置：
   - 步数、受阻、探索/轨迹/墙/障碍集合、左栏底图、当前位置；
   - 前端重新 `get_map()` 并重绘三栏。
3. **可见性**：顶栏显示当前种子；`get_state` 增加 `seed` 与 `strategy`。

## 非目标

- 不改迷宫生成算法、配色、视野、求解策略本身。
- 不做种子输入框、不做历史记录/持久化。
- 不改速度滑块与暂停逻辑。

## 实现要点

- `Solver.from_config(maze, cfg)`：统一构造入口，供 `main` 与 `Api.restart` 复用。
- `Solver.set_strategy(wall_follow, cluster_enabled)`：切换模式、清空路径；切到沿墙时重置 `facing/wall_turn/wall_states/wall_failed`，切到规划时 `mode=规划`。
- `Api`：
  - 保存 `cfg` 与当前 `strategy`；
  - `set_strategy(name)`、`get_strategy()`；
  - `restart()`：`cfg.seed = random(1, 2^31)` → `generate_maze` → 新 Solver → 返回新种子；
  - `get_state` 增加 `seed`、`strategy`。
- `__main__`：配置种子为 -1 时先取一个随机种子再生成，打印种子。
- 前端：
  - `boot()` 读取 `get_map()` 与 `get_strategy()`；
  - `reloadMap()`：重新拉地图、清空所有 Set、重置 bbox、重建底图；
  - 重开按钮：`await restart()` → `reloadMap()`；
  - 下拉 `onchange`：`set_strategy()`；
  - 顶栏统计显示「种子 | 步数 | 受阻 | 策略 | 状态」。

## 验收

- [ ] 下拉框有 3 个选项；切换后求解器行为随之变化（沿墙/规划模式可观测），无需重开地图。
- [ ] 点击重开：种子变化、地图变化、步数与受阻归零、三栏重绘；连续 3 次重开的种子与地图均不同。
- [ ] `get_state` 含 `seed` 与 `strategy`；重开后 `get_map` 返回新网格。
- [ ] 浏览器无 JS 报错；顶栏种子/策略显示正确。
- [ ] `py_compile` 通过，`uv run maze` 可启动。

## 产物路径

- `src/maze/solver.py`（`from_config`、`set_strategy`）
- `src/maze/visual.py`（下拉/重开 UI、Api 方法、重载地图、统计）
- `src/maze/__main__.py`（随机种子初始化与打印、复用 `from_config`）
- `config.toml` 不变

## 红线

- 不修改 `.venv/`、不全局安装依赖。
- 不把 `.parallel-agent/` 元文件混入业务提交。
- 没有命令与输出证据，不宣称完成。
