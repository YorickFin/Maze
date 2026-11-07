"""迷宫生成模块。

生成 w×h 网格迷宫：起点固定在左边界、终点固定在右边界（均垂直居中），
在可行区域内随机散布障碍块。可行动区域（空地）最小 1×1，障碍块尺寸在
1×1 到 (w/25)×(h/25) 之间随机。障碍总量由目标覆盖率 obstacle_ratio 控制，
避免过密堵死，同时受 obstacle_count 上限约束。
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
    obstacle_ratio: float = 0.15  # 障碍目标覆盖率
    seed: int = -1
    view_size: int = 10
    step_interval: float = 0.002
    max_steps: int = 500_000
    cell_px: int = 2

    @property
    def max_block_w(self) -> int:
        """障碍块最大宽度（格）= w/25，向下取整，至少 1。"""
        return max(1, self.width // 25)

    @property
    def max_block_h(self) -> int:
        """障碍块最大高度（格）= h/25，向下取整，至少 1。"""
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


def _scatter_obstacles(rng: random.Random, cfg: Config, grid: list[list[int]],
                       start: tuple[int, int], end: tuple[int, int]):
    """随机散布障碍块，覆盖率达到目标 ratio 或块数达到 count 上限。

    块尺寸在 1×1 到 (w/25)×(h/25) 之间随机，不覆盖起点/终点。
    """
    total = cfg.width * cfg.height
    target_walls = int(total * cfg.obstacle_ratio)
    wall_count = 0
    placed = 0
    attempts = 0
    max_attempts = max(2000, cfg.obstacle_count * 50)
    while placed < cfg.obstacle_count and wall_count < target_walls and attempts < max_attempts:
        attempts += 1
        bw = rng.randint(1, cfg.max_block_w)
        bh = rng.randint(1, cfg.max_block_h)
        bx = rng.randint(0, cfg.width - 1)
        by = rng.randint(0, cfg.height - 1)
        cells = []
        for dy in range(bh):
            for dx in range(bw):
                x, y = bx + dx, by + dy
                if x >= cfg.width or y >= cfg.height:
                    continue
                cells.append((x, y))
        if start in cells or end in cells:
            continue
        new_cells = []
        for (x, y) in cells:
            if grid[y][x] == EMPTY:
                new_cells.append((x, y))
        if not new_cells:
            continue
        for (x, y) in new_cells:
            grid[y][x] = WALL
        wall_count += len(new_cells)
        placed += 1


def generate_maze(cfg: Config) -> Maze:
    """按配置生成迷宫（随机散布障碍块）。"""
    rng = random.Random(cfg.seed if cfg.seed >= 0 else None)
    grid = [[EMPTY] * cfg.width for _ in range(cfg.height)]
    start = (0, cfg.height // 2)
    end = (cfg.width - 1, cfg.height // 2)

    _scatter_obstacles(rng, cfg, grid, start, end)
    # 确保起终点可通行
    grid[start[1]][start[0]] = EMPTY
    grid[end[1]][end[0]] = EMPTY

    return Maze(cfg.width, cfg.height, grid, start, end)
