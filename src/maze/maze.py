"""迷宫生成模块（房间式）。

把 w×h 网格按固定间隔划分成矩形房间网格（房间边长 ≤ (w/25)×(h/25)），
相邻房间之间隔 1 格厚墙。用随机生成树（Kruskal + 并查集）保证所有房间
连通（从而起点到终点必有至少一条路径）：生成树边强制开洞，非生成树边
按 corridor_extra_prob 概率额外开洞，形成「部分墙封闭」的房间式迷宫。

房间之间通道（门）宽度在 corridor_min~corridor_max 之间随机。
起点固定在左边界、终点固定在右边界（均垂直居中）。
"""

from __future__ import annotations

import random
from dataclasses import dataclass


@dataclass
class Config:
    """运行时配置（从 config.toml 解析）。"""

    width: int = 1000
    height: int = 1000
    obstacle_count: int = 3000
    obstacle_ratio: float = 0.15  # 兼容旧字段，房间式不用
    seed: int = -1
    view_size: int = 10
    step_interval: float = 0.002
    max_steps: int = 500_000
    cell_px: int = 2
    corridor_min: int = 1   # 通道最小宽度
    corridor_max: int = 5   # 通道最大宽度
    corridor_extra_prob: float = 0.4  # 非生成树边额外开洞概率（0~1）

    @property
    def max_block_w(self) -> int:
        """房间最大宽度（格）= w/25，向下取整，至少 1。"""
        return max(1, self.width // 25)

    @property
    def max_block_h(self) -> int:
        """房间最大高度（格）= h/25，向下取整，至少 1。"""
        return max(1, self.height // 25)


# 格状态常量
EMPTY = 0
WALL = 1


class Maze:
    """迷宫网格。

    grid[y][x] 取值：EMPTY=可通行，WALL=障碍。
    """

    def __init__(self, width: int, height: int, grid: list[list[int]],
                 start: tuple[int, int], end: tuple[int, int]):
        self.width = width
        self.height = height
        self.grid = grid
        self.start = start
        self.end = end

    def is_wall(self, x: int, y: int) -> bool:
        """越界视为墙，其余按网格判断。"""
        if x < 0 or x >= self.width or y < 0 or y >= self.height:
            return True
        return self.grid[y][x] == WALL

    def in_bounds(self, x: int, y: int) -> bool:
        return 0 <= x < self.width and 0 <= y < self.height

    def count_walls(self) -> int:
        return sum(1 for row in self.grid for c in row if c == WALL)


class _DSU:
    """并查集（房间连通性校验用）。"""

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


def _room_layout(cfg: Config):
    """计算房间网格布局（房间铺满整个网格，最后房间可变宽高）。

    返回 (cols, rows, room_w, room_h, step_w, step_h)。
    房间 (r,c) 的 x 范围 [c*step_w, min((c+1)*step_w-1, w-1)]，
    y 范围 [r*step_h, min((r+1)*step_h-1, h-1)]；房间间墙在
    x=(c+1)*step_w 与 y=(r+1)*step_h 处（若 < 网格尺寸）。
    """
    room_w = cfg.max_block_w
    room_h = cfg.max_block_h
    step_w = room_w + 1
    step_h = room_h + 1
    cols = max(1, -(-cfg.width // step_w))   # ceil
    rows = max(1, -(-cfg.height // step_h))  # ceil
    return cols, rows, room_w, room_h, step_w, step_h


def _build_rooms_grid(cfg: Config, cols, rows, room_w, room_h, step_w, step_h):
    """构建房间网格：房间内 EMPTY，房间间墙 WALL。

    房间 (r,c) 的右墙在 x=(c+1)*step_w（若 < 宽），下墙在 y=(r+1)*step_h。
    网格四周不额外设墙（边界视为墙）。最后房间铺满至网格边缘。
    """
    grid = [[EMPTY] * cfg.width for _ in range(cfg.height)]
    # 房间间竖墙：x = (c+1)*step_w - 1（房间 c 右侧墙），c=0..cols-2
    for c in range(cols - 1):
        wx = (c + 1) * step_w - 1
        if wx < cfg.width:
            for y in range(cfg.height):
                grid[y][wx] = WALL
    # 房间间横墙：y = (r+1)*step_h - 1（房间 r 下侧墙），r=0..rows-2
    for r in range(rows - 1):
        wy = (r + 1) * step_h - 1
        if wy < cfg.height:
            for x in range(cfg.width):
                grid[wy][x] = WALL
    return grid


def _room_rect(c, r, cfg, cols, rows, room_w, room_h, step_w, step_h):
    """房间 (r,c) 的网格范围 (x0,y0,x1,y1)（含）。"""
    x0 = c * step_w
    y0 = r * step_h
    x1 = min((c + 1) * step_w - 1, cfg.width - 1) if c == cols - 1 else c * step_w + room_w - 1
    y1 = min((r + 1) * step_h - 1, cfg.height - 1) if r == rows - 1 else r * step_h + room_h - 1
    return x0, y0, x1, y1


def _open_corridors(rng: random.Random, cfg: Config, grid,
                    cols, rows, room_w, room_h, step_w, step_h):
    """Kruskal 随机生成树开洞，保证房间连通。

    每条墙边：生成树边强制开洞；非树边按 corridor_extra_prob 概率开洞。
    洞（门）宽 corridor_min~corridor_max，位置沿墙随机。
    """
    # 收集所有相邻房间边（右邻居、下邻居）
    edges = []
    for r in range(rows):
        for c in range(cols):
            if c + 1 < cols:
                edges.append((r, c, r, c + 1))
            if r + 1 < rows:
                edges.append((r, c, r + 1, c))
    rng.shuffle(edges)

    dsu = _DSU()
    hole_len_choices = list(range(cfg.corridor_min, max(cfg.corridor_min, cfg.corridor_max) + 1))

    for (r1, c1, r2, c2) in edges:
        a = r1 * cols + c1
        b = r2 * cols + c2
        must_open = dsu.union(a, b)  # True=之前未连通（生成树边）
        if not must_open and rng.random() > cfg.corridor_extra_prob:
            continue  # 非树边且不额外开 → 保留墙
        hole_len = rng.choice(hole_len_choices)

        if c1 != c2:
            # 左右相邻 → 竖墙在 x = max(c1,c2)*step_w - 1，洞沿 y
            wx = max(c1, c2) * step_w - 1
            # 两房间同一行 r1，y 范围取两房间的交（同 r，相同 y 范围）
            _x0, y0, _x1, y1 = _room_rect(c1, r1, cfg, cols, rows, room_w, room_h, step_w, step_h)
            hole_len = min(hole_len, y1 - y0 + 1)
            hy = rng.randint(y0, y1 - hole_len + 1)
            for dy in range(hole_len):
                if 0 <= hy + dy < cfg.height and 0 <= wx < cfg.width:
                    grid[hy + dy][wx] = EMPTY
        else:
            # 上下相邻 → 横墙在 y = max(r1,r2)*step_h - 1，洞沿 x
            wy = max(r1, r2) * step_h - 1
            x0, _y0, x1, _y1 = _room_rect(c1, r1, cfg, cols, rows, room_w, room_h, step_w, step_h)
            hole_len = min(hole_len, x1 - x0 + 1)
            hx = rng.randint(x0, x1 - hole_len + 1)
            for dx in range(hole_len):
                if 0 <= hx + dx < cfg.width and 0 <= wy < cfg.height:
                    grid[wy][hx + dx] = EMPTY


def generate_maze(cfg: Config) -> Maze:
    """按配置生成房间式迷宫（房间网格 + 生成树保证连通）。"""
    rng = random.Random(cfg.seed if cfg.seed >= 0 else None)
    cols, rows, room_w, room_h, step_w, step_h = _room_layout(cfg)
    grid = _build_rooms_grid(cfg, cols, rows, room_w, room_h, step_w, step_h)
    _open_corridors(rng, cfg, grid, cols, rows, room_w, room_h, step_w, step_h)

    start = (0, cfg.height // 2)
    end = (cfg.width - 1, cfg.height // 2)
    # 确保起点/终点可通行（起点在左边界，可能在房间内或墙上）
    grid[start[1]][start[0]] = EMPTY
    grid[end[1]][end[0]] = EMPTY

    return Maze(cfg.width, cfg.height, grid, start, end)
