"""pywebview 可视化模块。

三栏渲染（参考示例图配色：黑=墙，白=可通行，灰=障碍，蓝=起点，红=终点）：
- 右栏：完整地图（含墙、灰色障碍、起点、终点）。
- 中栏：以破解方为中心的 view_size×view_size 实时视野，障碍不可见。
- 左栏：已探索合成图（走过格 + 推断障碍）。

前端通过 pywebview.api.get_state() 轮询状态，get_state() 返回 JSON 字符串。
"""

from __future__ import annotations

import json

from .maze import Config, Maze
from .solver import KNOWN_WALL, Solver

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
  <div class="panel"><h3>左 · 已探索合成图（黑=未探索 白=已探索 灰=推断障碍）</h3><div class="wrap"><canvas id="c-left"></canvas></div></div>
  <div class="panel"><h3>中 · 实时视野 (VIEWSIZE×VIEWSIZE, 障碍不可见)</h3><div class="wrap"><canvas id="c-mid"></canvas></div></div>
  <div class="panel"><h3>右 · 完整地图（黑=墙 白=可通行 灰=障碍）</h3><div class="wrap"><canvas id="c-right"></canvas></div></div>
</div>
<script>
const cell = 16;                // 中栏每格像素（10×10 → 160px）
const leftCell = 8;             // 左栏每格像素
const fullCell = 1;             // 右栏完整地图每格像素（W 格 → W px）
const vs = VIEWSIZE;
const C_WALL = '#000000';       // 墙体：黑
const C_EMPTY = '#ffffff';      // 可通行：白
const C_OBS = '#808080';        // 障碍：灰
const C_START = '#0070ff';      // 起点：蓝
const C_END = '#ff2d2d';        // 终点：红
const C_POS = '#ffd400';        // 破解方：黄
const C_TRAIL = 'rgba(60,140,255,0.45)';
let speed = 50;
let paused = false;

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

function drawRight(st) {
  const W = st.width, H = st.height;
  cRight.width = W * fullCell;
  cRight.height = H * fullCell;
  const ctx = cRight.getContext('2d');
  const img = ctx.createImageData(W, H);
  const d = img.data;
  for (let y = 0; y < H; y++) {
    const row = st.grid[y];
    for (let x = 0; x < W; x++) {
      const i = (y * W + x) * 4;
      const v = row[x];
      if (v === 1) { d[i]=0; d[i+1]=0; d[i+2]=0; }
      else if (v === 2) { d[i]=128; d[i+1]=128; d[i+2]=128; }
      else { d[i]=255; d[i+1]=255; d[i+2]=255; }
      d[i+3]=255;
    }
  }
  ctx.putImageData(img, 0, 0);
  // 已探索轨迹（半透明蓝）
  ctx.fillStyle = C_TRAIL;
  for (const v of st.visited) ctx.fillRect(v[0]*fullCell, v[1]*fullCell, fullCell, fullCell);
  // 起点蓝 / 终点红
  dot(ctx, st.start[0], st.start[1], 7, C_START);
  dot(ctx, st.end[0], st.end[1], 7, C_END);
  // 破解方当前位置（黄）
  dot(ctx, st.pos[0], st.pos[1], 4, C_POS);
}

function drawMid(st) {
  const px = st.pos[0], py = st.pos[1];
  const half = Math.floor(vs / 2);
  cMid.width = vs * cell;
  cMid.height = vs * cell;
  const ctx = cMid.getContext('2d');
  // 视野内障碍不可见：底色统一为白（可通行色）
  ctx.fillStyle = C_EMPTY;
  ctx.fillRect(0, 0, cMid.width, cMid.height);
  for (let vy = 0; vy < vs; vy++) {
    for (let vx = 0; vx < vs; vx++) {
      const gx = px - half + vx;
      const gy = py - half + vy;
      const key = gx + ',' + gy;
      // 已走过轨迹（蓝）
      if (st.visitedSet.has(key)) {
        ctx.fillStyle = C_TRAIL;
        ctx.fillRect(vx*cell, vy*cell, cell, cell);
      }
      // 推断障碍（灰，仅因受阻确认，属"推断"而非"看见"）
      if (st.knownWalls.has(key)) {
        ctx.fillStyle = C_OBS;
        ctx.fillRect(vx*cell, vy*cell, cell, cell);
      }
      ctx.strokeStyle = '#e6e6e6';
      ctx.strokeRect(vx*cell, vy*cell, cell, cell);
    }
  }
  // 起终点（若在视野内）：蓝 / 红
  const marker = (gx, gy, color) => {
    const cx = (gx - px + half) * cell + cell / 2;
    const cy = (gy - py + half) * cell + cell / 2;
    if (cx >= 0 && cx <= cMid.width && cy >= 0 && cy <= cMid.height) {
      ctx.beginPath(); ctx.arc(cx, cy, cell * 0.45, 0, Math.PI * 2);
      ctx.fillStyle = color; ctx.fill();
    }
  };
  marker(st.start[0], st.start[1], C_START);
  marker(st.end[0], st.end[1], C_END);
  // 破解方位置（中心，黄 + 黑描边）
  ctx.fillStyle = C_POS;
  ctx.fillRect(half*cell, half*cell, cell, cell);
  ctx.strokeStyle = C_WALL;
  ctx.strokeRect(half*cell, half*cell, cell, cell);
  ctx.strokeStyle = C_WALL;
  ctx.lineWidth = 2;
  ctx.strokeRect(0, 0, cMid.width, cMid.height);
}

function drawLeft(st) {
  const W = st.width, H = st.height;
  let minX = W, minY = H, maxX = 0, maxY = 0;
  const inr = (x,y) => x>=minX && x<=maxX && y>=minY && y<=maxY;
  // 探索范围以「视野扫过」explored 为准
  for (const k of st.explored) {
    const p = k.split(',').map(Number);
    if (p[0]<minX) minX=p[0]; if (p[0]>maxX) maxX=p[0];
    if (p[1]<minY) minY=p[1]; if (p[1]>maxY) maxY=p[1];
  }
  for (const k of st.knownWalls) {
    const p = k.split(',').map(Number);
    if (p[0]<minX) minX=p[0]; if (p[0]>maxX) maxX=p[0];
    if (p[1]<minY) minY=p[1]; if (p[1]>maxY) maxY=p[1];
  }
  const pad = 8;
  minX = Math.max(0, minX - pad); minY = Math.max(0, minY - pad);
  maxX = Math.min(W - 1, maxX + pad); maxY = Math.min(H - 1, maxY + pad);
  const gw = maxX - minX + 1, gh = maxY - minY + 1;
  cLeft.width = gw * leftCell;
  cLeft.height = gh * leftCell;
  const ctx = cLeft.getContext('2d');
  // 未探索区域：黑（与墙体同色）
  ctx.fillStyle = C_WALL;
  ctx.fillRect(0, 0, cLeft.width, cLeft.height);
  // 已感知区域（视野扫过）：白（可通行色）
  ctx.fillStyle = C_EMPTY;
  for (const k of st.explored) {
    const p = k.split(',').map(Number);
    ctx.fillRect((p[0]-minX)*leftCell, (p[1]-minY)*leftCell, leftCell, leftCell);
  }
  // 实际走过的路径：半透明蓝
  ctx.fillStyle = C_TRAIL;
  for (const v of st.visited) {
    ctx.fillRect((v[0]-minX)*leftCell, (v[1]-minY)*leftCell, leftCell, leftCell);
  }
  // 推断障碍：灰
  ctx.fillStyle = C_OBS;
  for (const k of st.knownWalls) {
    const p = k.split(',').map(Number);
    ctx.fillRect((p[0]-minX)*leftCell, (p[1]-minY)*leftCell, leftCell, leftCell);
  }
  if (inr(st.start[0],st.start[1])) dot(ctx,(st.start[0]-minX)*leftCell+leftCell/2,(st.start[1]-minY)*leftCell+leftCell/2,leftCell*0.7,C_START);
  if (inr(st.end[0],st.end[1])) dot(ctx,(st.end[0]-minX)*leftCell+leftCell/2,(st.end[1]-minY)*leftCell+leftCell/2,leftCell*0.7,C_END);
  dot(ctx,(st.pos[0]-minX)*leftCell+leftCell/2,(st.pos[1]-minY)*leftCell+leftCell/2,leftCell*0.6,C_POS);
}

async function tick() {
  if (!paused) {
    const st = JSON.parse(await window.pywebview.api.get_state());
    // Python 返回数组，转成 Set 供 includes/has 使用
    st.visitedSet = new Set(st.visitedSet || []);
    st.knownWalls = new Set(st.knownWalls || []);
    st.exploredSet = new Set(st.explored || []);
    drawRight(st); drawMid(st); drawLeft(st);
    document.getElementById('stats').textContent =
      '步数 ' + st.steps + ' | 受阻 ' + st.blocked + ' | 状态：' + st.status;
    if (st.status !== '完成' && st.status !== '卡死') {
      await window.pywebview.api.advance();
    }
  }
  requestAnimationFrame(tick);
}

document.getElementById('btn-pause').onclick = function() {
  paused = !paused;
  this.textContent = paused ? '继续' : '暂停';
};
document.getElementById('speed').oninput = function() {
  speed = parseInt(this.value);
  window.pywebview.api.set_speed(speed);
};

window.addEventListener('pywebviewready', function() { tick(); });
</script>
</body>
</html>
"""


def build_html(view_size: int) -> str:
    """生成带实际视距的 HTML。"""
    return _HTML_TEMPLATE.replace("VIEWSIZE", str(view_size))


class Api:
    """暴露给前端的 JS API。"""

    def __init__(self, maze: Maze, solver: Solver, cfg: Config):
        self.maze = maze
        self.solver = solver
        self.cfg = cfg
        self.speed = 50  # 1-100

    def get_state(self) -> str:
        s = self.solver
        st = {
            "width": self.maze.width,
            "height": self.maze.height,
            "grid": self.maze.grid,
            "start": list(self.maze.start),
            "end": list(self.maze.end),
            "pos": list(s.pos),
            "visited": [list(v) for v in s.visited],
            "visitedSet": [f"{v[0]},{v[1]}" for v in s.visited],
            "explored": [f"{c[0]},{c[1]}" for c in s.explored],
            "knownWalls": [f"{c[0]},{c[1]}" for c in s.known if s.known[c] == KNOWN_WALL],
            "steps": s.step_count,
            "blocked": s.blocked_hits,
            "status": "完成" if s.reached_end else ("卡死" if s.done else "探索中"),
        }
        return json.dumps(st)

    def advance(self) -> None:
        """按速度推进若干步。速度 1-100 → 单帧 1~200 步。"""
        n = 1 + self.speed * 2
        for _ in range(n):
            if not self.solver.step():
                break

    def set_speed(self, value: int) -> None:
        self.speed = int(value)
