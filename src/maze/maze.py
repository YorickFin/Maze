"""「有机形态」迷宫生成模块。

骨架是一组带抖动的节点阵列，相邻节点用 Kruskal 随机生成树连成
一条全连通骨架，再按 loop_prob 额外连边形成环路。沿每条骨架边用
「可变半径圆盘笔刷」填充通道，节点处填充更大的圆形腔室，因此最终
通道宽窄不一、转角圆润、节点处膨大，接近手绘迷宫。

生成后在通道内投放灰色障碍斑块（OBSTACLE，阻挡移动）。为不切断
起点→终点通路，障碍只落在「起点→终点最短路径」的保护带之外。

格状态：EMPTY=0 可通行，WALL=1 墙，OBSTACLE=2 障碍。
起点在左边界垂直居中，终点在右边界垂直居中。
"""

from __future__ import annotations

import math
import random
from collections import deque
from dataclasses import dataclass


@dataclass
class Config:
    """运行时配置（从 config.toml 解析）。"""

    width: int = 1000
    height: int = 1000
    seed: int = -1
    node_spacing: int = 100     # 节点阵列基础间距（格）
    node_jitter: int = 25       # 节点位置抖动（格）
    corridor_radius_min: int = 13  # 通道最小半径（格）
    corridor_radius_max: int = 20  # 通道最大半径（格）
    chamber_radius_min: int = 22   # 节点腔室最小半径（格）
    chamber_radius_max: int = 40   # 节点腔室最大半径（格）
    chamber_prob: float = 0.55     # 节点膨大腔室出现概率（0~1）
    loop_prob: float = 0.12        # 非生成树边额外连边概率（0~1）
    obstacle_blobs: int = 12       # 灰色障碍斑块数量
    obstacle_radius_min: int = 12  # 障碍斑块最小半径（格）
    obstacle_radius_max: int = 26  # 障碍斑块最大半径（格）
    view_size: int = 25
    step_interval: float = 0.002
    max_steps: int = 500_000
    cost_right: float = 1.0        # 向右移动代价
    cost_vertical: float = 3.0     # 向上/下移动代价
    cost_left: float = 10.0        # 向左回退代价
    suspect_penalty: float = 20.0  # 穿过「疑似障碍」格的附加代价
    plan_expand_limit: int = 200_000  # 单次规划最大扩展格数
    wall_follow: bool = True       # 是否启用「沿墙为主」模式
    cell_px: int = 2


# 格状态常量
EMPTY = 0
WALL = 1
OBSTACLE = 2


class Maze:
    """迷宫网格。

    grid[y][x] 取值：EMPTY=可通行，WALL=墙，OBSTACLE=障碍。
    """

    def __init__(self, width: int, height: int, grid: list[list[int]],
                 start: tuple[int, int], end: tuple[int, int]):
        self.width = width
        self.height = height
        self.grid = grid
        self.start = start
        self.end = end

    def is_wall(self, x: int, y: int) -> bool:
        """是否墙体（越界视为墙）。"""
        if x < 0 or x >= self.width or y < 0 or y >= self.height:
            return True
        return self.grid[y][x] == WALL

    def is_blocked(self, x: int, y: int) -> bool:
        """是否不可通行（墙或灰色障碍，越界视为不可通行）。"""
        if x < 0 or x >= self.width or y < 0 or y >= self.height:
            return True
        return self.grid[y][x] != EMPTY

    def sense(self, x: int, y: int):
        """传感器读数：墙与可通行可见，灰色障碍不可见（返回 None）。

        返回 WALL / EMPTY / None（越界或灰色障碍不可见）。
        """
        if not self.in_bounds(x, y):
            return None
        v = self.grid[y][x]
        if v == WALL:
            return WALL
        if v == OBSTACLE:
            return None
        return EMPTY

    def is_goal_marker(self, x: int, y: int) -> bool:
        """终点标记（红点）是否出现在该格（视野内可见）。"""
        return (x, y) == self.end

    def in_bounds(self, x: int, y: int) -> bool:
        return 0 <= x < self.width and 0 <= y < self.height

    def count_walls(self) -> int:
        return sum(1 for row in self.grid for c in row if c == WALL)

    def count_obstacles(self) -> int:
        return sum(1 for row in self.grid for c in row if c == OBSTACLE)

    def count_empty(self) -> int:
        return sum(1 for row in self.grid for c in row if c == EMPTY)


class _DSU:
    """并查集（节点连通性校验用）。"""

    def __init__(self):
        self.parent: dict[int, int] = {}

    def find(self, x: int) -> int:
        self.parent.setdefault(x, x)
        root = x
        while self.parent[root] != root:
            root = self.parent[root]
        while self.parent[x] != root:
            self.parent[x], x = root, self.parent[x]
        return root

    def union(self, a: int, b: int) -> bool:
        ra, rb = self.find(a), self.find(b)
        if ra == rb:
            return False
        self.parent[ra] = rb
        return True


def _fill_disk(grid, w: int, h: int, cx: float, cy: float,
               radius: float, value: int) -> None:
    """以 (cx,cy) 为圆心、radius 为半径填充圆盘（逐行整段赋值，速度快）。"""
    if radius <= 0:
        return
    r2 = radius * radius
    y0 = max(0, int(math.floor(cy - radius)))
    y1 = min(h - 1, int(math.ceil(cy + radius)))
    for y in range(y0, y1 + 1):
        dy = y - cy
        if dy * dy > r2:
            continue
        half = math.sqrt(r2 - dy * dy)
        xa = max(0, int(math.ceil(cx - half)))
        xb = min(w - 1, int(math.floor(cx + half)))
        if xa <= xb:
            grid[y][xa:xb + 1] = [value] * (xb - xa + 1)


def _carve_path(grid, w: int, h: int, p0, p1, r_start: float, r_end: float,
                rng: random.Random, wobble: float) -> None:
    """沿两点间的二次贝塞尔曲线填充可变半径通道。

    wobble 控制弯曲幅度（相对长度）；半径沿路径线性过渡并叠加正弦抖动，
    使通道边缘宽窄不一。
    """
    x0, y0 = p0
    x1, y1 = p1
    dx, dy = x1 - x0, y1 - y0
    length = math.hypot(dx, dy)
    if length < 1.0:
        _fill_disk(grid, w, h, x0, y0, max(r_start, r_end), EMPTY)
        return

    # 垂直方向偏移，形成弯曲通道
    perp_x, perp_y = -dy / length, dx / length
    bend = rng.uniform(-wobble, wobble) * length
    cx = (x0 + x1) / 2 + perp_x * bend
    cy = (y0 + y1) / 2 + perp_y * bend

    steps = max(2, int(length / 1.5))
    phase = rng.uniform(0, math.tau)
    amp = rng.uniform(0.6, 1.8)
    for i in range(steps + 1):
        t = i / steps
        u = 1.0 - t
        x = u * u * x0 + 2 * u * t * cx + t * t * x1
        y = u * u * y0 + 2 * u * t * cy + t * t * y1
        radius = r_start * u + r_end * t + amp * math.sin(phase + t * math.tau * 1.1)
        _fill_disk(grid, w, h, x, y, max(3.0, radius), EMPTY)


def _make_nodes(cfg: Config, rng: random.Random):
    """生成带抖动的节点阵列，返回 (nodes, cols, rows)。

    node = (x, y, 腔室半径)。
    """
    margin = cfg.chamber_radius_max + 6
    usable_w = max(1, cfg.width - 2 * margin)
    usable_h = max(1, cfg.height - 2 * margin)
    cols = max(2, round(usable_w / cfg.node_spacing) + 1)
    rows = max(2, round(usable_h / cfg.node_spacing) + 1)
    step_x = usable_w / (cols - 1)
    step_y = usable_h / (rows - 1)
    jitter_x = min(cfg.node_jitter, step_x / 3)
    jitter_y = min(cfg.node_jitter, step_y / 3)
    max_x = cfg.width - margin - 1
    max_y = cfg.height - margin - 1

    nodes: list[tuple[float, float, float]] = []
    for j in range(rows):
        for i in range(cols):
            x = margin + i * step_x + rng.uniform(-jitter_x, jitter_x)
            y = margin + j * step_y + rng.uniform(-jitter_y, jitter_y)
            x = min(max(x, margin), max_x)
            y = min(max(y, margin), max_y)
            # 部分节点膨大成腔室，其余只保留通道宽度，避免出现"串珠"感
            if rng.random() < cfg.chamber_prob:
                r = rng.uniform(cfg.chamber_radius_min, cfg.chamber_radius_max)
            else:
                r = rng.uniform(cfg.corridor_radius_min, cfg.corridor_radius_max)
            nodes.append((x, y, r))
    return nodes, cols, rows


def _carve_skeleton(cfg: Config, rng: random.Random, grid,
                    nodes, cols: int, rows: int) -> None:
    """节点腔室 + Kruskal 生成树连边 + 额外环路。"""
    # 节点腔室：先填充，作为通道交汇处的膨大部分
    for (x, y, r) in nodes:
        _fill_disk(grid, cfg.width, cfg.height, x, y, r, EMPTY)

    # 候选边：右、下、右下（抖动后呈不规则走向）
    edges: list[tuple[int, int]] = []
    for j in range(rows):
        for i in range(cols):
            idx = j * cols + i
            if i + 1 < cols:
                edges.append((idx, idx + 1))
            if j + 1 < rows:
                edges.append((idx, idx + cols))
            if i + 1 < cols and j + 1 < rows:
                edges.append((idx, idx + cols + 1))
    rng.shuffle(edges)

    dsu = _DSU()
    for a, b in edges:
        is_tree = dsu.union(a, b)  # True=首次连通，即生成树边
        if not is_tree and rng.random() >= cfg.loop_prob:
            continue
        xa, ya, _ = nodes[a]
        xb, yb, _ = nodes[b]
        ra = rng.uniform(cfg.corridor_radius_min, cfg.corridor_radius_max)
        rb = rng.uniform(cfg.corridor_radius_min, cfg.corridor_radius_max)
        _carve_path(grid, cfg.width, cfg.height, (xa, ya), (xb, yb),
                    ra, rb, rng, wobble=0.18)


def _connect_edges(cfg: Config, rng: random.Random, grid, nodes,
                   start: tuple[int, int], end: tuple[int, int]) -> None:
    """把起点/终点从左右边界接入最近的节点（近乎水平的短通道）。"""
    sx, sy = start
    ex, ey = end
    left_idx = min(range(len(nodes)),
                   key=lambda k: (nodes[k][0] - sx) ** 2 + (nodes[k][1] - sy) ** 2)
    right_idx = min(range(len(nodes)),
                    key=lambda k: (nodes[k][0] - ex) ** 2 + (nodes[k][1] - ey) ** 2)
    lx, ly, _ = nodes[left_idx]
    rx, ry, _ = nodes[right_idx]

    r = rng.uniform(cfg.corridor_radius_min, cfg.corridor_radius_max)
    _carve_path(grid, cfg.width, cfg.height, (sx, sy), (lx, ly),
                r, r * 0.9, rng, wobble=0.04)
    _carve_path(grid, cfg.width, cfg.height, (ex, ey), (rx, ry),
                r, r * 0.9, rng, wobble=0.04)


def _shortest_path(grid, w: int, h: int, start, end):
    """BFS 求起点→终点最短路径（返回格序列，不含路径则为空列表）。"""
    n = w * h
    parent = [-1] * n
    si = start[1] * w + start[0]
    ei = end[1] * w + end[0]
    parent[si] = si
    queue = deque([si])
    while queue:
        cur = queue.popleft()
        if cur == ei:
            break
        cy, cx = divmod(cur, w)
        if cx > 0:
            nxt = cur - 1
            if parent[nxt] == -1 and grid[cy][cx - 1] == EMPTY:
                parent[nxt] = cur
                queue.append(nxt)
        if cx + 1 < w:
            nxt = cur + 1
            if parent[nxt] == -1 and grid[cy][cx + 1] == EMPTY:
                parent[nxt] = cur
                queue.append(nxt)
        if cy > 0:
            nxt = cur - w
            if parent[nxt] == -1 and grid[cy - 1][cx] == EMPTY:
                parent[nxt] = cur
                queue.append(nxt)
        if cy + 1 < h:
            nxt = cur + w
            if parent[nxt] == -1 and grid[cy + 1][cx] == EMPTY:
                parent[nxt] = cur
                queue.append(nxt)

    if parent[ei] == -1:
        return []
    path = []
    cur = ei
    while cur != si:
        path.append((cur % w, cur // w))
        cur = parent[cur]
    path.append(start)
    path.reverse()
    return path


def _build_protected(path, w: int, h: int, radius: float) -> set[int]:
    """沿最短路径铺一圈保护带（键 = y*w+x），障碍不得落在这里。"""
    protected: set[int] = set()
    r2 = radius * radius
    step = 2
    for i in range(0, len(path), step):
        cx, cy = path[i]
        y0 = max(0, int(math.floor(cy - radius)))
        y1 = min(h - 1, int(math.ceil(cy + radius)))
        for y in range(y0, y1 + 1):
            dy = y - cy
            if dy * dy > r2:
                continue
            half = math.sqrt(r2 - dy * dy)
            xa = max(0, int(math.ceil(cx - half)))
            xb = min(w - 1, int(math.floor(cx + half)))
            base = y * w
            for x in range(xa, xb + 1):
                protected.add(base + x)
    return protected


def _has_clearance(grid, w: int, h: int, cx: float, cy: float,
                   radius: float) -> bool:
    """检查中心周围一圈是否全是可通行格（让灰斑完整落在白色区域内）。"""
    for k in range(12):
        a = k * math.pi / 6
        x = int(round(cx + math.cos(a) * radius))
        y = int(round(cy + math.sin(a) * radius))
        if not (0 <= x < w and 0 <= y < h) or grid[y][x] != EMPTY:
            return False
    return True


def _add_obstacles(cfg: Config, rng: random.Random, grid, start, end,
                   protected: set[int]) -> None:
    """在通道内投放灰色障碍斑块（椭圆，按通道形状裁剪）。"""
    w, h = cfg.width, cfg.height
    placed = 0
    attempts = 0
    max_attempts = cfg.obstacle_blobs * 120
    while placed < cfg.obstacle_blobs and attempts < max_attempts:
        attempts += 1
        cx = rng.randrange(2, w - 2)
        cy = rng.randrange(2, h - 2)
        if grid[cy][cx] != EMPTY:
            continue
        if cy * w + cx in protected:
            continue
        if abs(cx - start[0]) + abs(cy - start[1]) < 60:
            continue
        if abs(cx - end[0]) + abs(cy - end[1]) < 60:
            continue

        rx = rng.uniform(cfg.obstacle_radius_min, cfg.obstacle_radius_max)
        ry = rng.uniform(cfg.obstacle_radius_min, cfg.obstacle_radius_max)
        if not _has_clearance(grid, w, h, cx, cy, max(rx, ry) + 3):
            continue  # 白色区域不够大，换一处
        angle = rng.uniform(0, math.pi)
        painted = _fill_blob(grid, w, h, cx, cy, rx, ry, angle, protected)
        if painted:
            placed += 1


def _fill_blob(grid, w: int, h: int, cx: float, cy: float, rx: float, ry: float,
               angle: float, protected: set[int]) -> int:
    """填充旋转椭圆斑块，只覆盖可通行格且避开保护带。返回涂改格数。"""
    ca, sa = math.cos(angle), math.sin(angle)
    rmax = max(rx, ry)
    y0 = max(0, int(math.floor(cy - rmax)))
    y1 = min(h - 1, int(math.ceil(cy + rmax)))
    painted = 0
    for y in range(y0, y1 + 1):
        dy = y - cy
        row = grid[y]
        x0 = max(0, int(math.floor(cx - rmax)))
        x1 = min(w - 1, int(math.ceil(cx + rmax)))
        base = y * w
        for x in range(x0, x1 + 1):
            dx = x - cx
            u = dx * ca + dy * sa
            v = -dx * sa + dy * ca
            if (u * u) / (rx * rx) + (v * v) / (ry * ry) <= 1.0:
                if row[x] == EMPTY and base + x not in protected:
                    row[x] = OBSTACLE
                    painted += 1
    return painted


def generate_maze(cfg: Config) -> Maze:
    """按配置生成有机形态迷宫（节点生成树 + 圆盘笔刷 + 灰色障碍斑块）。"""
    rng = random.Random(cfg.seed if cfg.seed >= 0 else None)
    grid = [[WALL] * cfg.width for _ in range(cfg.height)]

    nodes, cols, rows = _make_nodes(cfg, rng)
    start = (0, cfg.height // 2)
    end = (cfg.width - 1, cfg.height // 2)

    _carve_skeleton(cfg, rng, grid, nodes, cols, rows)
    _connect_edges(cfg, rng, grid, nodes, start, end)

    # 起终点处补圆点，保证落在通道内
    _fill_disk(grid, cfg.width, cfg.height, start[0], start[1], 5.0, EMPTY)
    _fill_disk(grid, cfg.width, cfg.height, end[0], end[1], 5.0, EMPTY)

    # 保护起点→终点最短路径，障碍不会切断主通路
    path = _shortest_path(grid, cfg.width, cfg.height, start, end)
    margin = cfg.obstacle_radius_max + 3
    protected = _build_protected(path, cfg.width, cfg.height, margin) if path else set()
    _add_obstacles(cfg, rng, grid, start, end, protected)

    grid[start[1]][start[0]] = EMPTY
    grid[end[1]][end[0]] = EMPTY
    return Maze(cfg.width, cfg.height, grid, start, end)
