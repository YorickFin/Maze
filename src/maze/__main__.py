"""主入口：读配置 → 生成迷宫 → 构建求解器 → 启动 pywebview 可视化。"""

from __future__ import annotations

import sys
import tomllib
from pathlib import Path

import webview

from .maze import Config, generate_maze
from .solver import Solver
from .visual import Api, build_html

CONFIG_PATH = Path(__file__).resolve().parent.parent.parent / "config.toml"


def load_config(path: Path = CONFIG_PATH) -> Config:
    """从 config.toml 读取配置。"""
    with open(path, "rb") as f:
        data = tomllib.load(f)

    maze_cfg = data.get("maze", {})
    obs = data.get("obstacle", {})
    solver_cfg = data.get("solver", {})
    vis = data.get("visual", {})

    return Config(
        width=int(maze_cfg.get("width", 1000)),
        height=int(maze_cfg.get("height", 1000)),
        obstacle_count=int(obs.get("count", 3000)),
        obstacle_ratio=float(obs.get("ratio", 0.15)),
        seed=int(obs.get("seed", -1)),
        view_size=int(solver_cfg.get("view_size", 10)),
        step_interval=float(solver_cfg.get("step_interval", 0.002)),
        max_steps=int(solver_cfg.get("max_steps", 500_000)),
        cell_px=int(vis.get("cell_px", 2)),
    )


def main() -> None:
    cfg = load_config()
    maze = generate_maze(cfg)
    solver = Solver(maze, view_size=cfg.view_size, max_steps=cfg.max_steps)

    # 简单预检查：起点是否被障碍覆盖
    if maze.is_wall(*maze.start):
        print(f"警告：起点 {maze.start} 被障碍覆盖，跳过。")
        sys.exit(1)

    print(f"迷宫 {maze.width}x{maze.height}，障碍格 {maze.count_walls()}，"
          f"起点 {maze.start}，终点 {maze.end}，视距 {cfg.view_size}x{cfg.view_size}")

    api = Api(maze, solver, cfg)
    html = build_html(cfg.view_size)

    window = webview.create_window(
        "迷宫 A* 破解演示",
        html=html,
        js_api=api,
        width=1200,
        height=700,
    )
    webview.start()


if __name__ == "__main__":
    main()
