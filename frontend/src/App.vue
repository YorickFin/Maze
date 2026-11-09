<script setup lang="ts">
import { computed, onMounted, onUnmounted, reactive, ref } from 'vue'

type Vec = [number, number]

interface MazeMap {
  width: number
  height: number
  grid: number[][]
  start: Vec
  end: Vec
  path?: Vec[]
  view_size?: number
}

interface MazeState {
  pos: Vec
  steps: number
  blocked: number
  mode: string
  seed: number
  strategy: string
  view_size?: number
  move_mode?: string
  status: string
  newExplored: string[]
  newVisited: string[]
  newWalls: string[]
  newObstacles: string[]
}

const CELL = 16
const C_WALL = '#000000'
const C_EMPTY = '#ffffff'
const C_OBS = '#808080'
const C_START = '#0070ff'
const C_END = '#ff2d2d'
const C_POS = '#ffd400'
const C_TRAIL = '#4a9eff'
const C_PATH = '#a855f7'
const STRATEGY_LABEL: Record<string, string> = {
  cluster: '聚类 gain',
  wall: '沿墙',
  frontier: '前沿规划',
}

const cRight = ref<HTMLCanvasElement | null>(null)
const cMid = ref<HTMLCanvasElement | null>(null)
const cLeft = ref<HTMLCanvasElement | null>(null)

const strategy = ref('cluster')
const speed = ref(50)
const viewSize = ref(25)
const moveMode = ref('8')
const paused = ref(false)
const started = ref(false)
const reloading = ref(false)
const stats = reactive({
  seed: '-',
  steps: 0,
  blocked: 0,
  strategy: 'cluster',
  status: '探索中',
})

let MAP: MazeMap | null = null
let rightBase: HTMLCanvasElement | null = null
let leftBase: HTMLCanvasElement | null = null
let leftCtx: CanvasRenderingContext2D | null = null
let bbox: { minX: number; minY: number; maxX: number; maxY: number } | null = null
let busy = false
let running = true

const explored = new Set<string>()
const visited = new Set<string>()
const walls = new Set<string>()
const obstacles = new Set<string>()
const trace: Vec[] = []

const statusClass = computed(() => {
  if (stats.status === '完成') return 'state done'
  if (stats.status === '卡死') return 'state stuck'
  return 'state'
})

const getApi = () => (window as any).pywebview?.api

function dot(ctx: CanvasRenderingContext2D, x: number, y: number, r: number, color: string) {
  const w = ctx.canvas.width
  const h = ctx.canvas.height
  const px = Math.min(Math.max(r, x), w - r)
  const py = Math.min(Math.max(r, y), h - r)
  ctx.beginPath()
  ctx.arc(px, py, r, 0, Math.PI * 2)
  ctx.fillStyle = color
  ctx.fill()
}

function cellFill(ctx: CanvasRenderingContext2D, key: string, color: string) {
  const [x, y] = key.split(',').map(Number)
  ctx.fillStyle = color
  ctx.fillRect(x, y, 1, 1)
}

function expandBBox(key: string) {
  const [x, y] = key.split(',').map(Number)
  if (!bbox) {
    bbox = { minX: x, minY: y, maxX: x, maxY: y }
    return
  }
  if (x < bbox.minX) bbox.minX = x
  if (y < bbox.minY) bbox.minY = y
  if (x > bbox.maxX) bbox.maxX = x
  if (y > bbox.maxY) bbox.maxY = y
}

function buildBase() {
  if (!MAP) return
  const W = MAP.width
  const H = MAP.height
  rightBase = document.createElement('canvas')
  rightBase.width = W
  rightBase.height = H
  const bctx = rightBase.getContext('2d')!
  const img = bctx.createImageData(W, H)
  const d = img.data
  for (let y = 0; y < H; y++) {
    const row = MAP.grid[y]
    for (let x = 0; x < W; x++) {
      const i = (y * W + x) * 4
      const v = row[x]
      if (v === 1) {
        d[i] = 0; d[i + 1] = 0; d[i + 2] = 0
      } else if (v === 2) {
        d[i] = 128; d[i + 1] = 128; d[i + 2] = 128
      } else {
        d[i] = 255; d[i + 1] = 255; d[i + 2] = 255
      }
      d[i + 3] = 255
    }
  }
  bctx.putImageData(img, 0, 0)

  // 完整地图上的最短路径（紫线，仅换图/首次启动时随底图重绘）
  const path = MAP.path ?? []
  if (path.length > 1) {
    bctx.strokeStyle = C_PATH
    bctx.lineWidth = 3
    bctx.lineJoin = 'round'
    bctx.beginPath()
    bctx.moveTo(path[0][0] + 0.5, path[0][1] + 0.5)
    for (let i = 1; i < path.length; i++) {
      bctx.lineTo(path[i][0] + 0.5, path[i][1] + 0.5)
    }
    bctx.stroke()
  }

  leftBase = document.createElement('canvas')
  leftBase.width = W
  leftBase.height = H
  leftCtx = leftBase.getContext('2d')!
  leftCtx.fillStyle = C_WALL
  leftCtx.fillRect(0, 0, W, H)
}

function mergeState(st: MazeState) {
  if (!leftCtx) return
  if (st.newExplored.length) {
    for (const k of st.newExplored) {
      if (!explored.has(k)) {
        explored.add(k)
        expandBBox(k)
      }
      cellFill(leftCtx, k, C_EMPTY)
    }
  }
  if (st.newWalls.length) {
    for (const k of st.newWalls) {
      walls.add(k)
      expandBBox(k)
      cellFill(leftCtx, k, C_WALL)
    }
  }
  if (st.newObstacles.length) {
    for (const k of st.newObstacles) {
      obstacles.add(k)
      expandBBox(k)
      cellFill(leftCtx, k, C_OBS)
    }
  }
  if (st.newVisited.length) {
    for (const k of st.newVisited) {
      visited.add(k)
      expandBBox(k)
      cellFill(leftCtx, k, C_TRAIL)
      const [x, y] = k.split(',').map(Number)
      trace.push([x, y])
    }
  }
}

function drawRight(st: MazeState) {
  const el = cRight.value
  if (!el || !MAP || !rightBase) return
  el.width = MAP.width
  el.height = MAP.height
  const ctx = el.getContext('2d')!
  ctx.drawImage(rightBase, 0, 0)
  ctx.fillStyle = C_TRAIL
  for (const k of visited) {
    const [x, y] = k.split(',').map(Number)
    ctx.fillRect(x, y, 2, 2)
  }
  // 依次连接访问点（含对角），让八向移动的轨迹连续
  if (trace.length > 1) {
    ctx.strokeStyle = C_TRAIL
    ctx.lineWidth = 1
    for (let i = 1; i < trace.length; i++) {
      const [x0, y0] = trace[i - 1]
      const [x1, y1] = trace[i]
      ctx.beginPath()
      ctx.moveTo(x0 + 0.5, y0 + 0.5)
      ctx.lineTo(x1 + 0.5, y1 + 0.5)
      ctx.stroke()
    }
  }
  dot(ctx, MAP.start[0], MAP.start[1], 10, C_START)
  dot(ctx, MAP.end[0], MAP.end[1], 10, C_END)
  const px = Math.min(Math.max(8, st.pos[0]), MAP.width - 8)
  const py = Math.min(Math.max(8, st.pos[1]), MAP.height - 8)
  ctx.beginPath()
  ctx.arc(px, py, 8, 0, Math.PI * 2)
  ctx.lineWidth = 3
  ctx.strokeStyle = C_POS
  ctx.stroke()
}

function drawMid(st: MazeState) {
  const el = cMid.value
  if (!el || !MAP) return
  const vs = st.view_size ?? MAP.view_size ?? 25
  const half = Math.floor(vs / 2)
  const px = st.pos[0]
  const py = st.pos[1]
  el.width = vs * CELL
  el.height = vs * CELL
  const ctx = el.getContext('2d')!
  ctx.fillStyle = C_EMPTY
  ctx.fillRect(0, 0, el.width, el.height)
  for (let vy = 0; vy < vs; vy++) {
    for (let vx = 0; vx < vs; vx++) {
      const gx = px - half + vx
      const gy = py - half + vy
      const key = `${gx},${gy}`
      if (gx < 0 || gx >= MAP.width || gy < 0 || gy >= MAP.height || walls.has(key)) {
        ctx.fillStyle = C_WALL
        ctx.fillRect(vx * CELL, vy * CELL, CELL, CELL)
      } else if (obstacles.has(key)) {
        ctx.fillStyle = C_OBS
        ctx.fillRect(vx * CELL, vy * CELL, CELL, CELL)
      }
      if (visited.has(key)) {
        ctx.fillStyle = C_TRAIL
        ctx.fillRect(vx * CELL, vy * CELL, CELL, CELL)
      }
      ctx.strokeStyle = '#e6e6e6'
      ctx.strokeRect(vx * CELL, vy * CELL, CELL, CELL)
    }
  }
  const marker = (gx: number, gy: number, color: string) => {
    const cx = (gx - px + half) * CELL + CELL / 2
    const cy = (gy - py + half) * CELL + CELL / 2
    if (cx >= 0 && cx <= el.width && cy >= 0 && cy <= el.height) {
      ctx.beginPath()
      ctx.arc(cx, cy, CELL * 0.45, 0, Math.PI * 2)
      ctx.fillStyle = color
      ctx.fill()
    }
  }
  ctx.fillStyle = C_POS
  ctx.fillRect(half * CELL, half * CELL, CELL, CELL)
  ctx.strokeStyle = C_WALL
  ctx.strokeRect(half * CELL, half * CELL, CELL, CELL)
  marker(MAP.start[0], MAP.start[1], C_START)
  marker(MAP.end[0], MAP.end[1], C_END)
  ctx.lineWidth = 2
  ctx.strokeRect(0, 0, el.width, el.height)
}

function drawLeft(st: MazeState) {
  const el = cLeft.value
  if (!el || !MAP || !leftBase || !bbox) return
  const pad = 8
  const minX = Math.max(0, bbox.minX - pad)
  const minY = Math.max(0, bbox.minY - pad)
  const maxX = Math.min(MAP.width - 1, bbox.maxX + pad)
  const maxY = Math.min(MAP.height - 1, bbox.maxY + pad)
  const gw = maxX - minX + 1
  const gh = maxY - minY + 1
  const scale = Math.max(1, Math.min(8, Math.floor(760 / Math.max(gw, gh))))
  el.width = gw * scale
  el.height = gh * scale
  const ctx = el.getContext('2d')!
  ctx.imageSmoothingEnabled = false
  ctx.drawImage(leftBase, minX, minY, gw, gh, 0, 0, gw * scale, gh * scale)

  const marker = (gx: number, gy: number, color: string, r: number) => {
    if (gx < minX || gx > maxX || gy < minY || gy > maxY) return
    ctx.beginPath()
    ctx.arc(
      (gx - minX) * scale + scale / 2,
      (gy - minY) * scale + scale / 2,
      Math.max(2, scale * r),
      0,
      Math.PI * 2,
    )
    ctx.fillStyle = color
    ctx.fill()
  }
  marker(MAP.start[0], MAP.start[1], C_START, 0.7)
  marker(MAP.end[0], MAP.end[1], C_END, 0.7)
  marker(st.pos[0], st.pos[1], C_POS, 0.6)
}

async function tick() {
  if (!running) return
  requestAnimationFrame(tick)
  const api = getApi()
  if (!MAP || !api || paused.value || busy || reloading.value) return
  busy = true
  try {
    const st: MazeState = JSON.parse(await api.get_state())
    mergeState(st)
    drawRight(st)
    drawMid(st)
    drawLeft(st)
    stats.seed = String(st.seed)
    stats.steps = st.steps
    stats.blocked = st.blocked
    stats.strategy = st.strategy
    stats.status = st.status
    if (started.value && st.status !== '完成' && st.status !== '卡死') {
      await api.advance()
    }
  } finally {
    busy = false
  }
}

function resetView() {
  explored.clear()
  visited.clear()
  walls.clear()
  obstacles.clear()
  trace.length = 0
  bbox = null
}

async function boot() {
  const api = getApi()
  if (!api) return
  const data: MazeMap = JSON.parse(await api.get_map())
  MAP = data
  strategy.value = await api.get_strategy()
  viewSize.value = data.view_size ?? 25
  moveMode.value = await api.get_move_mode()
  buildBase()
  tick()
}

function togglePause() {
  paused.value = !paused.value
}

function onSpeed() {
  getApi()?.set_speed(speed.value)
}

function onViewSize() {
  getApi()?.set_view_size(viewSize.value)
}

function onMoveMode() {
  getApi()?.set_move_mode(moveMode.value)
  trace.length = 0
}

function onStrategy() {
  getApi()?.set_strategy(strategy.value)
}

async function start() {
  const api = getApi()
  if (!api || reloading.value) return
  reloading.value = true
  try {
    // 不换图：回到起点，重置已探索与实时视野后重新跑
    await api.reset()
    resetView()
    if (MAP) buildBase()
    stats.steps = 0
    stats.blocked = 0
    stats.status = '探索中'
    paused.value = false
    started.value = true
  } finally {
    reloading.value = false
  }
}

async function newMap() {
  const api = getApi()
  if (!api || reloading.value) return
  reloading.value = true
  try {
    await api.restart()
    resetView()
    MAP = JSON.parse(await api.get_map())
    buildBase()
    paused.value = false
    started.value = false // 换图后等点击「开始」
  } finally {
    reloading.value = false
  }
}

onMounted(() => {
  if (getApi()) {
    boot()
  } else {
    window.addEventListener('pywebviewready', boot, { once: true })
  }
})

onUnmounted(() => {
  running = false
})
</script>

<template>
  <div class="app">
    <header class="topbar">
      <div class="brand">
        <div class="logo">M</div>
        <div class="brand-text">
          <h1>MAZE</h1>
          <p>迷宫破解演示</p>
        </div>
      </div>

      <div class="controls">
        <button class="primary" :disabled="reloading" @click="start">开始</button>
        <button class="ghost" :disabled="reloading" @click="newMap">换图</button>
        <button class="ghost" :disabled="!started" @click="togglePause">{{ paused ? '继续' : '暂停' }}</button>
        <label class="field">
          <span>策略</span>
          <select v-model="strategy" @change="onStrategy">
            <option value="cluster">聚类 gain</option>
            <option value="wall">沿墙</option>
            <option value="frontier">前沿规划</option>
          </select>
        </label>
        <label class="field">
          <span>移动</span>
          <select v-model="moveMode" @change="onMoveMode">
            <option value="8">8 向</option>
            <option value="4">4 向</option>
          </select>
        </label>
        <label class="field slider">
          <span>速度</span>
          <input v-model.number="speed" type="range" min="1" max="100" @input="onSpeed" />
        </label>
        <label class="field slider">
          <span>视野 {{ viewSize }}</span>
          <input v-model.number="viewSize" type="range" min="10" max="100" @input="onViewSize" />
        </label>
      </div>

      <div class="status">
        <span class="chip">种子 {{ stats.seed }}</span>
        <span class="chip">步数 {{ stats.steps }}</span>
        <span class="chip">受阻 {{ stats.blocked }}</span>
        <span class="chip">移动 {{ moveMode }} 向</span>
        <span class="chip accent">{{ STRATEGY_LABEL[stats.strategy] || stats.strategy }}</span>
        <span :class="statusClass">{{ stats.status }}</span>
      </div>
    </header>

    <main class="grid">
      <section class="card">
        <div class="card-head">
          <h2>已探索</h2>
          <span class="hint">黑=墙/未知 · 白=通道 · 灰=推断障碍</span>
        </div>
        <div class="canvas-wrap"><canvas ref="cLeft" /></div>
      </section>
      <section class="card">
        <div class="card-head">
          <h2>实时视野</h2>
          <span class="hint">墙可见 · 障碍不可见</span>
        </div>
        <div class="canvas-wrap"><canvas ref="cMid" /></div>
      </section>
      <section class="card">
        <div class="card-head">
          <h2>完整地图</h2>
          <span class="hint">黑=墙 · 白=可通行 · 灰=障碍 · 紫=最短路径</span>
        </div>
        <div class="canvas-wrap"><canvas ref="cRight" /></div>
      </section>
    </main>
  </div>
</template>
