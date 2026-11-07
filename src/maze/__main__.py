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
    gen = data.get("generator", {})
    obs = data.get("obstacle", {})
    solver_cfg = data.get("solver", {})
    vis = data.get("visual", {})

    return Config(
        width=int(maze_cfg.get("width", 1000)),
        height=int(maze_cfg.get("height", 1000)),
        seed=int(gen.get("seed", -1)),
        node_spacing=int(gen.get("node_spacing", 100)),
        node_jitter=int(gen.get("node_jitter", 25)),
        corridor_radius_min=int(gen.get("corridor_radius_min", 13)),
        corridor_radius_max=int(gen.get("corridor_radius_max", 20)),
        chamber_radius_min=int(gen.get("chamber_radius_min", 22)),
        chamber_radius_max=int(gen.get("chamber_radius_max", 40)),
        chamber_prob=float(gen.get("chamber_prob", 0.55)),
        loop_prob=float(gen.get("loop_prob", 0.12)),
        obstacle_blobs=int(obs.get("blobs", 12)),
        obstacle_radius_min=int(obs.get("radius_min", 12)),
        obstacle_radius_max=int(obs.get("radius_max", 26)),
        view_size=int(solver_cfg.get("view_size", 25)),
        step_interval=float(solver_cfg.get("step_interval", 0.002)),
        max_steps=int(solver_cfg.get("max_steps", 500_000)),
        cost_right=float(solver_cfg.get("cost_right", 1.0)),
        cost_vertical=float(solver_cfg.get("cost_vertical", 3.0)),
        cost_left=float(solver_cfg.get("cost_left", 10.0)),
        suspect_penalty=float(solver_cfg.get("suspect_penalty", 20.0)),
        plan_expand_limit=int(solver_cfg.get("plan_expand_limit", 200_000)),
        wall_follow=bool(solver_cfg.get("wall_follow", True)),
        cell_px=int(vis.get("cell_px", 2)),
    )


def main() -> None:
    cfg = load_config()
    maze = generate_maze(cfg)
    solver = Solver(
        maze,
        view_size=cfg.view_size,
        max_steps=cfg.max_steps,
        cost_right=cfg.cost_right,
        cost_vertical=cfg.cost_vertical,
        cost_left=cfg.cost_left,
        suspect_penalty=cfg.suspect_penalty,
        plan_expand_limit=cfg.plan_expand_limit,
        wall_follow=cfg.wall_follow,
    )

    # 简单预检查：起点是否被墙或障碍覆盖
    if maze.is_blocked(*maze.start):
        print(f"警告：起点 {maze.start} 不可通行，跳过。")
        sys.exit(1)

    print(f"迷宫 {maze.width}x{maze.height}，墙格 {maze.count_walls()}，"
          f"障碍格 {maze.count_obstacles()}，通道格 {maze.count_empty()}，"
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
