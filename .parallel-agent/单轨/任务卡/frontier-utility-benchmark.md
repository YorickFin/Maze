---
kind: 任务卡
task_id: frontier-utility-benchmark
mode: L2
---

# 任务卡：前沿聚类 / 效用 frontier 离线对比（视野 10 vs 25）

## 目标

参考 GitHub 开源实现（WFD 聚类、效用 frontier、最近 frontier），在**不改业务代码**的前提下做离线原型对比，回答：

1. 前沿聚类 + 小簇过滤能否减少“碎前沿抖动”；
2. 效用函数（信息增益 + 右向进度 − 路径代价）能否让路径更接近最优；
3. 不同方案/混合方案（沿墙、聚类代价、聚类效用、沿墙兜底）的差异；
4. 视野 10 与 25 对结果的影响。

最终由用户决定是否落地到项目。

## 参考仓库

- SeanReg/nav2_wavefront_frontier_exploration（WFD：波前沿 BFS、聚类、过滤小簇、质心）
- HanwenCao/Frontier_Exploration（效用 frontier：Shannon−Renyi 信息增益）
- sdwyc/frontier_exploration_occupancygrid（occupancy grid + 质心）
- tyuownu/nearest_frontier_planner（最近 frontier）
- paulbovbel/frontier_exploration（经典 frontier costmap）

## 对比方案

- **A 沿墙**：当前实现（沿墙为主 + 规划兜底）。
- **B 聚类代价**：frontier 聚类 + 过滤小簇，按路径代价选最近簇。
- **C 聚类效用**：frontier 聚类 + 过滤小簇，按 `utility = 信息增益 + 右向进度 − 路径代价` 选簇。
- **C2/C3 效用变体**：加大右向权重、加大信息增益权重、加大代价权重。
- **混合**：C/C2 为主 + 无有效簇时回退（沿墙/前沿规划）；沿墙为主 + 打转时切 C。
- 视野：10×10、25×25 各跑一遍。

## 指标

- 到达终点（pos == maze.end）
- 步数、步数 / 全图最优步数（倍率）
- 墙钟耗时、受阻次数
- 兜底触发次数（无有效前沿簇时）
- 8 个种子（42/7/99/123/2024/5/88/77）对比表

## 非目标

- 不修改 `src/`、`config.toml`、界面。
- 不做 ROS 集成、不引入新依赖。
- 不验证 60 视野方案（本轮只按用户要求测 10/25）。

## 验收

- [ ] 8 种子 × 方案 × 视野的对比表（到达、步数、倍率、耗时、受阻、兜底）
- [ ] 给出“哪种方案最稳、哪种最接近最优、视野 10/25 差异”的结论
- [ ] 结论写入台账；由用户决定是否实施

## 产物

- 仅临时脚本（存放在系统临时目录，不进仓库）
- 台账条目
