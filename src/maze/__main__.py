"""主入口：读配置 → 生成迷宫 → 构建求解器 → 启动 pywebview 可视化。"""

from __future__ import annotations

import functools
import os
import random
import sys
import threading
import tomllib
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import webview

from .maze import Config, generate_maze
from .solver import Solver
from .visual import Api

ROOT = Path(__file__).resolve().parent.parent.parent
CONFIG_PATH = ROOT / "config.toml"
FRONTEND_DIST = ROOT / "frontend" / "dist"


class _QuietHandler(SimpleHTTPRequestHandler):
    """静态文件服务，不打印每条请求日志。"""

    def log_message(self, fmt: str, *args) -> None:
        pass


def _serve_frontend() -> str:
    """在本地随机端口托管 frontend/dist，返回 index.html 的 URL。"""
    handler = functools.partial(_QuietHandler, directory=str(FRONTEND_DIST))
    server = ThreadingHTTPServer(("127.0.0.1", 0), handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    port = server.server_address[1]
    return f"http://127.0.0.1:{port}/index.html"


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
        wall_follow=bool(solver_cfg.get("wall_follow", False)),
        cluster_enabled=bool(solver_cfg.get("cluster_enabled", True)),
        cluster_gain_weight=float(solver_cfg.get("cluster_gain_weight", 2.0)),
        cluster_right_weight=float(solver_cfg.get("cluster_right_weight", 0.0)),
        cluster_cost_weight=float(solver_cfg.get("cluster_cost_weight", 1.0)),
        cluster_min_size=int(solver_cfg.get("cluster_min_size", 5)),
        cluster_top_k=int(solver_cfg.get("cluster_top_k", 4)),
        cluster_gain_radius=int(solver_cfg.get("cluster_gain_radius", 10)),
        cluster_expand_limit=int(solver_cfg.get("cluster_expand_limit", 10_000)),
        cell_px=int(vis.get("cell_px", 2)),
    )


def main() -> None:
    cfg = load_config()
    if cfg.seed < 0:
        cfg.seed = random.randrange(1, 2**31)
    maze = generate_maze(cfg)
    solver = Solver.from_config(maze, cfg)

    # 简单预检查：起点是否被墙或障碍覆盖
    if maze.is_blocked(*maze.start):
        print(f"警告：起点 {maze.start} 不可通行，跳过。")
        sys.exit(1)

    print(f"迷宫 {maze.width}x{maze.height}，种子 {cfg.seed}，"
          f"墙格 {maze.count_walls()}，"
          f"障碍格 {maze.count_obstacles()}，通道格 {maze.count_empty()}，"
          f"起点 {maze.start}，终点 {maze.end}，视距 {cfg.view_size}x{cfg.view_size}")

    api = Api(maze, solver, cfg)
    url = os.environ.get("MAZE_FRONTEND_URL")
    if not url:
        index = FRONTEND_DIST / "index.html"
        if not index.exists():
            print(f"未找到前端构建产物：{index}")
            print("请先运行：cd frontend && pnpm build")
            sys.exit(1)
        url = _serve_frontend()

    window = webview.create_window(
        "迷宫破解演示",
        url=url,
        js_api=api,
        width=1280,
        height=800,
    )
    webview.start()


if __name__ == "__main__":
    main()
