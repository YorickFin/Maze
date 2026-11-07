"""pywebview 可视化模块。

三栏渲染（黑=墙，白=可通行，灰=障碍，蓝=起点，红=终点）：
- 右栏：完整地图（含墙、灰色障碍、起点、终点），底图只在启动时绘制一次。
- 中栏：以破解方为中心的 view_size×view_size 实时视野；墙可见、通道可见，
  灰色障碍不可见（只有被「受阻」确认后才显示为灰）。
- 左栏：已探索合成图（白色已探索、黑色已知墙、灰色推断障碍、蓝色轨迹）。

状态传输：`get_map()` 只在启动时返回一次全图；`get_state()` 之后只返回
位置/步数/受阻/状态与增量集合，避免每帧重传 1000×1000 网格。
"""

from __future__ import annotations

import json

from .maze import Config, Maze
from .solver import Solver

_HTML_TEMPLATE = """<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<style>
  body { margin: 0; background: #111; color: #ddd; font-family: sans-serif; overflow: hidden; }
  #topbar { height: 42px; display: flex; align-items: center; gap: 14px; padding: 0 14px;
            background: #1b1b1b; font-size: 13px; }
  #topbar button { background: #2b2b2b; color: #ddd; border: 1px solid #444;
                   padding: 5px 12px; border-radius: 4px; cursor: pointer; font-size: 13px; }
  #topbar button:hover { background: #3a3a3a; }
  #topbar label { display: flex; align-items: center; gap: 6px; }
  #topbar input[type=range] { width: 140px; }
  #stats { margin-left: auto; color: #8f8; }
  #panels { display: flex; height: calc(100vh - 42px); }
  .panel { flex: 1; display: flex; flex-direction: column; min-width: 0; }
  .panel h3 { margin: 6px 10px; font-size: 13px; color: #aaa; font-weight: 500; }
  .panel .wrap { flex: 1; overflow: auto; display: flex; align-items: center; justify-content: center; }
  canvas { background: #000; display: block; max-width: 100%; max-height: 100%; }
</style>
</head>
<body>
<div id="topbar">
  <button id="btn-pause">暂停</button>
  <label>速度 <input id="speed" type="range" min="1" max="100" value="50"></label>
  <span id="stats">步数 0 | 受阻 0 | 状态：探索中</span>
</div>
<div id="panels">
  <div class="panel"><h3>左 · 已探索合成图（黑=墙/未探索 白=通道 灰=推断障碍）</h3><div class="wrap"><canvas id="c-left"></canvas></div></div>
  <div class="panel"><h3>中 · 实时视野 (VIEWSIZE×VIEWSIZE, 墙可见 障碍不可见)</h3><div class="wrap"><canvas id="c-mid"></canvas></div></div>
  <div class="panel"><h3>右 · 完整地图（黑=墙 白=可通行 灰=障碍）</h3><div class="wrap"><canvas id="c-right"></canvas></div></div>
</div>
<script>
const CELL = 16;                 // 中栏每格像素
const VS = VIEWSIZE;             // 视野边长（格）
const C_WALL = '#000000';
const C_EMPTY = '#ffffff';
const C_OBS = '#808080';
const C_START = '#0070ff';
const C_END = '#ff2d2d';
const C_POS = '#ffd400';
const C_TRAIL = '#4a9eff';

let MAP = null;
let rightBase = null;
let leftBase = null;
let leftCtx = null;
let bbox = null;                 // 左栏绘制范围 {minX,minY,maxX,maxY}
let paused = false;
let busy = false;

const explored = new Set();
const visited = new Set();
const walls = new Set();
const obstacles = new Set();

const cRight = document.getElementById('c-right');
const cMid = document.getElementById('c-mid');
const cLeft = document.getElementById('c-left');

function dot(ctx, x, y, r, color) {
  const w = ctx.canvas.width, h = ctx.canvas.height;
  const px = Math.min(Math.max(r, x), w - r);
  const py = Math.min(Math.max(r, y), h - r);
  ctx.beginPath();
  ctx.arc(px, py, r, 0, Math.PI * 2);
  ctx.fillStyle = color;
  ctx.fill();
}

function cellFill(ctx, key, color) {
  const p = key.split(',');
  ctx.fillStyle = color;
  ctx.fillRect(+p[0], +p[1], 1, 1);
}

function expandBBox(key) {
  const p = key.split(',');
  const x = +p[0], y = +p[1];
  if (!bbox) { bbox = {minX: x, minY: y, maxX: x, maxY: y}; return; }
  if (x < bbox.minX) bbox.minX = x;
  if (y < bbox.minY) bbox.minY = y;
  if (x > bbox.maxX) bbox.maxX = x;
  if (y > bbox.maxY) bbox.maxY = y;
}

function buildBase() {
  const W = MAP.width, H = MAP.height;
  rightBase = document.createElement('canvas');
  rightBase.width = W;
  rightBase.height = H;
  const bctx = rightBase.getContext('2d');
  const img = bctx.createImageData(W, H);
  const d = img.data;
  for (let y = 0; y < H; y++) {
    const row = MAP.grid[y];
    for (let x = 0; x < W; x++) {
      const i = (y * W + x) * 4;
      const v = row[x];
      if (v === 1) { d[i] = 0; d[i+1] = 0; d[i+2] = 0; }
      else if (v === 2) { d[i] = 128; d[i+1] = 128; d[i+2] = 128; }
      else { d[i] = 255; d[i+1] = 255; d[i+2] = 255; }
      d[i+3] = 255;
    }
  }
  bctx.putImageData(img, 0, 0);

  leftBase = document.createElement('canvas');
  leftBase.width = W;
  leftBase.height = H;
  leftCtx = leftBase.getContext('2d');
  leftCtx.fillStyle = C_WALL;
  leftCtx.fillRect(0, 0, W, H);
}

function mergeState(st) {
  if (st.newExplored.length) {
    leftCtx.fillStyle = C_EMPTY;
    for (const k of st.newExplored) {
      if (!explored.has(k)) { explored.add(k); expandBBox(k); }
      cellFill(leftCtx, k, C_EMPTY);
    }
  }
  if (st.newWalls.length) {
    for (const k of st.newWalls) { walls.add(k); expandBBox(k); }
    leftCtx.fillStyle = C_WALL;
    for (const k of st.newWalls) cellFill(leftCtx, k, C_WALL);
  }
  if (st.newObstacles.length) {
    for (const k of st.newObstacles) { obstacles.add(k); expandBBox(k); }
    leftCtx.fillStyle = C_OBS;
    for (const k of st.newObstacles) cellFill(leftCtx, k, C_OBS);
  }
  if (st.newVisited.length) {
    for (const k of st.newVisited) { visited.add(k); expandBBox(k); }
    leftCtx.fillStyle = C_TRAIL;
    for (const k of st.newVisited) cellFill(leftCtx, k, C_TRAIL);
  }
}

function drawRight(st) {
  cRight.width = MAP.width;
  cRight.height = MAP.height;
  const ctx = cRight.getContext('2d');
  ctx.drawImage(rightBase, 0, 0);
  ctx.fillStyle = C_TRAIL;
  for (const k of visited) {
    const p = k.split(',');
    ctx.fillRect(+p[0], +p[1], 2, 2);  // 2px 保证缩略后仍可见
  }
  dot(ctx, MAP.start[0], MAP.start[1], 10, C_START);
  dot(ctx, MAP.end[0], MAP.end[1], 10, C_END);
  // 当前位置用空心黄环，避免盖住终点红点
  const px = Math.min(Math.max(8, st.pos[0]), MAP.width - 8);
  const py = Math.min(Math.max(8, st.pos[1]), MAP.height - 8);
  ctx.beginPath();
  ctx.arc(px, py, 8, 0, Math.PI * 2);
  ctx.lineWidth = 3;
  ctx.strokeStyle = C_POS;
  ctx.stroke();
}

function drawMid(st) {
  const half = Math.floor(VS / 2);
  const px = st.pos[0], py = st.pos[1];
  cMid.width = VS * CELL;
  cMid.height = VS * CELL;
  const ctx = cMid.getContext('2d');
  ctx.fillStyle = C_EMPTY;
  ctx.fillRect(0, 0, cMid.width, cMid.height);
  for (let vy = 0; vy < VS; vy++) {
    for (let vx = 0; vx < VS; vx++) {
      const gx = px - half + vx;
      const gy = py - half + vy;
      const key = gx + ',' + gy;
      if (gx < 0 || gx >= MAP.width || gy < 0 || gy >= MAP.height || walls.has(key)) {
        ctx.fillStyle = C_WALL;
        ctx.fillRect(vx*CELL, vy*CELL, CELL, CELL);
      } else if (obstacles.has(key)) {
        ctx.fillStyle = C_OBS;
        ctx.fillRect(vx*CELL, vy*CELL, CELL, CELL);
      }
      if (visited.has(key)) {
        ctx.fillStyle = C_TRAIL;
        ctx.fillRect(vx*CELL, vy*CELL, CELL, CELL);
      }
      ctx.strokeStyle = '#e6e6e6';
      ctx.strokeRect(vx*CELL, vy*CELL, CELL, CELL);
    }
  }
  const marker = (gx, gy, color) => {
    const cx = (gx - px + half) * CELL + CELL / 2;
    const cy = (gy - py + half) * CELL + CELL / 2;
    if (cx >= 0 && cx <= cMid.width && cy >= 0 && cy <= cMid.height) {
      ctx.beginPath(); ctx.arc(cx, cy, CELL * 0.45, 0, Math.PI * 2);
      ctx.fillStyle = color; ctx.fill();
    }
  };
  // 破解方位置（中心，黄 + 黑描边）
  ctx.fillStyle = C_POS;
  ctx.fillRect(half*CELL, half*CELL, CELL, CELL);
  ctx.strokeStyle = C_WALL;
  ctx.strokeRect(half*CELL, half*CELL, CELL, CELL);
  // 起终点标记最后绘制：站在红点上时仍能看到终点
  marker(MAP.start[0], MAP.start[1], C_START);
  marker(MAP.end[0], MAP.end[1], C_END);
  ctx.lineWidth = 2;
  ctx.strokeRect(0, 0, cMid.width, cMid.height);
}

function drawLeft(st) {
  if (!bbox) return;
  const pad = 8;
  const minX = Math.max(0, bbox.minX - pad);
  const minY = Math.max(0, bbox.minY - pad);
  const maxX = Math.min(MAP.width - 1, bbox.maxX + pad);
  const maxY = Math.min(MAP.height - 1, bbox.maxY + pad);
  const gw = maxX - minX + 1, gh = maxY - minY + 1;
  const scale = Math.max(1, Math.min(8, Math.floor(760 / Math.max(gw, gh))));
  cLeft.width = gw * scale;
  cLeft.height = gh * scale;
  const ctx = cLeft.getContext('2d');
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(leftBase, minX, minY, gw, gh, 0, 0, gw * scale, gh * scale);

  const marker = (gx, gy, color, r) => {
    if (gx < minX || gx > maxX || gy < minY || gy > maxY) return;
    ctx.beginPath();
    ctx.arc((gx - minX) * scale + scale / 2, (gy - minY) * scale + scale / 2,
            Math.max(2, scale * r), 0, Math.PI * 2);
    ctx.fillStyle = color;
    ctx.fill();
  };
  marker(MAP.start[0], MAP.start[1], C_START, 0.7);
  marker(MAP.end[0], MAP.end[1], C_END, 0.7);
  marker(st.pos[0], st.pos[1], C_POS, 0.6);
}

async function tick() {
  requestAnimationFrame(tick);
  if (!MAP || paused || busy) return;
  busy = true;
  try {
    const st = JSON.parse(await window.pywebview.api.get_state());
    mergeState(st);
    drawRight(st); drawMid(st); drawLeft(st);
    document.getElementById('stats').textContent =
      '步数 ' + st.steps + ' | 受阻 ' + st.blocked + ' | 状态：' + st.status;
    if (st.status !== '完成' && st.status !== '卡死') {
      await window.pywebview.api.advance();
    }
  } finally {
    busy = false;
  }
}

async function boot() {
  MAP = JSON.parse(await window.pywebview.api.get_map());
  buildBase();
  tick();
}

document.getElementById('btn-pause').onclick = function() {
  paused = !paused;
  this.textContent = paused ? '继续' : '暂停';
};
document.getElementById('speed').oninput = function() {
  window.pywebview.api.set_speed(parseInt(this.value));
};

window.addEventListener('pywebviewready', boot);
</script>
</body>
</html>
"""


def build_html(view_size: int) -> str:
    """生成带实际视距的 HTML。"""
    return _HTML_TEMPLATE.replace("VIEWSIZE", str(view_size))


def _key(cell: tuple[int, int]) -> str:
    return f"{cell[0]},{cell[1]}"


class Api:
    """暴露给前端的 JS API。"""

    def __init__(self, maze: Maze, solver: Solver, cfg: Config):
        self.maze = maze
        self.solver = solver
        self.cfg = cfg
        self.speed = 50  # 1-100

    def get_map(self) -> str:
        """全图只在启动时返回一次，之后由前端缓存。"""
        return json.dumps({
            "width": self.maze.width,
            "height": self.maze.height,
            "grid": self.maze.grid,
            "start": list(self.maze.start),
            "end": list(self.maze.end),
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
