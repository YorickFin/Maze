---
kind: 任务卡
task_id: solver-cluster-gain
mode: L3
---

# 任务卡：前沿聚类 + gain 效用（性能优化版）

## 目标

把对比中最优的「前沿聚类 + gain 权重」落地到 `solver.py`，同时解决原型 6~50 秒/种子的性能问题：

1. **增量 frontier 维护**
   - 初始观测后维护 `frontier` 集合；每次 `_observe` 只检查新观测格及其 4 邻居，O(1) 更新，不再全量扫 `known`。
2. **前沿聚类**
   - frontier 格按 4-邻接做连通聚类；过滤小于 5 的碎簇。
   - 每簇代表 = 簇内 x 最大的格（同 x 取离当前位置 y 最近）。
3. **效用选择**
   - `utility = 2.0 * gain + 0.0 * right − 1.0 * manhattan`
   - `gain` = 代表周围 21×21 内未观测格数量；`right` = 代表相对当前位置的 x 增量；`manhattan` 为路径长度估计。
   - 按 utility 排序，取前 3 个簇依次尝试。
4. **目标规划**
   - 对选中的簇代表用 A*（曼哈顿启发）在已知可通行格上规划；单次扩展上限可配（默认 30000）。
   - 成功后沿路径走到代表，再踏入其未知邻格（优先向右）。
   - 全部失败 → 回退到现有前沿规划（含向右收束），保证不卡死。
5. **模式与配置**
   - 沿墙模式保留，`wall_follow` 默认改为 false（聚类 gain 为主）。
   - 新增配置：`cluster_gain_weight`、`cluster_right_weight`、`cluster_cost_weight`、`cluster_min_size`、`cluster_top_k`、`cluster_gain_radius`、`cluster_expand_limit`。
6. **可观测性**
   - 顶栏策略显示：沿墙 / 规划（聚类仍在规划模式内，不新增状态字符串）。

## 非目标

- 不改视野（保持 25×25）、迷宫生成、配色、地图传输协议。
- 不做 60 视野方案、不做 ROS 集成、不引入新依赖。
- 不追求 seed=7 的个例最优；整体平均与稳定性优先。

## 未知项

无（方案与参数来自 S-20261107-011 的对比，用户已选定）。

## 假设

- 增量 frontier 与全量重算等价；用随机步全量重算校验。
- 曼哈顿路径估计可替代原型中的精确多目标 Dijkstra，用于效用排序；A* 只对最终候选算真实路径。
- A* 使用曼哈顿启发，边代价最小为 1，启发可采纳。

## 候选方案

已收敛，不再展开。

## 验收

- [ ] 8 个种子（42/7/99/123/2024/5/88/77）全部到达 `maze.end`。
- [ ] 输出每种子步数、倍率、耗时、受阻；8 种子平均倍率 ≤ 4.70×（沿墙基线），最差 ≤ 9.0×；单种子耗时 ≤ 5 秒。
- [ ] 增量 frontier 正确性：任选 3 个检查点，用全量 `known` 重算 frontier，与 `solver.frontier` 完全一致。
- [ ] 感知断言（seed=42）：视野内墙 ∈ 已知墙、通道 ∈ 已知可通行、障碍不在已知集合；`bad_perception=0`。
- [ ] 回退路径：无有效簇或 A* 失败时能回到现有前沿规划，8 种子无卡死。
- [ ] `get_state` 正常、顶栏无 JS 报错；`py_compile` 通过，`uv run maze` 可启动。
- [ ] `wall_follow=false` 为默认；改成 true 仍可回到沿墙模式。

## 产物路径

- `src/maze/solver.py`（增量 frontier、聚类、gain 效用、A* 到簇代表）
- `src/maze/maze.py`（Config 新增聚类参数，wall_follow 默认 false）
- `src/maze/visual.py`（如需要，仅显示策略；不改协议）
- `src/maze/__main__.py`（传递新参数）
- `config.toml`（聚类参数与默认值）

## 红线

- 不修改 `.venv/`、不全局安装依赖。
- 不把 `.parallel-agent/` 元文件混入业务提交。
- 没有命令与输出证据，不宣称完成。
