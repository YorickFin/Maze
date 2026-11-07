"""pywebview 的 JS API。

前端已迁移到 `frontend/`（Vue 3 + Vite）；本模块只暴露 Api，
不再包含内联 HTML 模板。
"""

from __future__ import annotations

import json
import random

from .maze import Config, Maze, generate_maze
from .solver import Solver


def _key(cell: tuple[int, int]) -> str:
    return f"{cell[0]},{cell[1]}"


class Api:
    """暴露给前端的 JS API。"""

    def __init__(self, maze: Maze, solver: Solver, cfg: Config):
        self.maze = maze
        self.solver = solver
        self.cfg = cfg
        self.speed = 50  # 1-100
        self.strategy = self._strategy_name(cfg)

    @staticmethod
    def _strategy_name(cfg: Config) -> str:
        if cfg.wall_follow:
            return "wall"
        if cfg.cluster_enabled:
            return "cluster"
        return "frontier"

    @staticmethod
    def _strategy_flags(name: str) -> tuple[bool, bool]:
        return {"cluster": (False, True),
                "wall": (True, False),
                "frontier": (False, False)}[name]

    def get_strategy(self) -> str:
        return self.strategy

    def set_strategy(self, name: str) -> None:
        if name not in ("cluster", "wall", "frontier"):
            return
        self.strategy = name
        self.solver.set_strategy(*self._strategy_flags(name))

    def set_view_size(self, value: int) -> None:
        """运行时修改视野边长（10~100）。"""
        value = max(10, min(100, int(value)))
        self.cfg.view_size = value
        self.solver.set_view_size(value)

    def restart(self) -> str:
        """换随机种子重新生成迷宫，并重建求解器。"""
        self.cfg.seed = random.randrange(1, 2**31)
        maze = generate_maze(self.cfg)
        solver = Solver.from_config(maze, self.cfg)
        solver.set_strategy(*self._strategy_flags(self.strategy))
        self.maze = maze
        self.solver = solver
        return json.dumps({"seed": self.cfg.seed})

    def get_map(self) -> str:
        """全图只在启动/重开时返回一次，之后由前端缓存。"""
        return json.dumps({
            "width": self.maze.width,
            "height": self.maze.height,
            "grid": self.maze.grid,
            "start": list(self.maze.start),
            "end": list(self.maze.end),
            "view_size": self.cfg.view_size,
        }, separators=(",", ":"))

    def get_state(self) -> str:
        """每帧只返回增量与标量，不再重传全图。"""
        s = self.solver
        if s.pos == self.maze.end:
            status = "完成"
        elif s.done:
            status = "卡死"
        else:
            status = "探索中"
        st = {
            "pos": list(s.pos),
            "steps": s.step_count,
            "blocked": s.blocked_hits,
            "mode": s.mode,
            "seed": self.cfg.seed,
            "strategy": self.strategy,
            "view_size": self.cfg.view_size,
            "status": status,
            "newExplored": [_key(c) for c in s.new_explored],
            "newVisited": [_key(c) for c in s.new_visited],
            "newWalls": [_key(c) for c in s.new_walls],
            "newObstacles": [_key(c) for c in s.new_obstacles],
        }
        s.new_explored.clear()
        s.new_visited.clear()
        s.new_walls.clear()
        s.new_obstacles.clear()
        return json.dumps(st, separators=(",", ":"))

    def advance(self) -> None:
        """按速度推进若干步；到达终点即停。"""
        n = 1 + self.speed * 2
        for _ in range(n):
            if self.solver.pos == self.maze.end:
                break
            if not self.solver.step():
                break

    def set_speed(self, value: int) -> None:
        self.speed = int(value)
