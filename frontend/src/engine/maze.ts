/** 浏览器版有机迷宫生成（与 src/maze/maze.py 同算法的 TS 移植）。 */
import { DEFAULT_CONFIG, type MazeConfig } from './config'
import { Rng } from './rng'

export const EMPTY = 0
export const WALL = 1
export const OBSTACLE = 2

function fillDisk(
  grid: Uint8Array,
  w: number,
  h: number,
  cx: number,
  cy: number,
  radius: number,
  value: number,
): void {
  if (radius <= 0) return
  const r2 = radius * radius
  const y0 = Math.max(0, Math.floor(cy - radius))
  const y1 = Math.min(h - 1, Math.ceil(cy + radius))
  for (let y = y0; y <= y1; y++) {
    const dy = y - cy
    if (dy * dy > r2) continue
    const half = Math.sqrt(r2 - dy * dy)
    const xa = Math.max(0, Math.ceil(cx - half))
    const xb = Math.min(w - 1, Math.floor(cx + half))
    if (xa <= xb) grid.fill(value, y * w + xa, y * w + xb + 1)
  }
}

function carvePath(
  grid: Uint8Array,
  w: number,
  h: number,
  p0: [number, number],
  p1: [number, number],
  rStart: number,
  rEnd: number,
  rng: Rng,
  wobble: number,
): void {
  const [x0, y0] = p0
  const [x1, y1] = p1
  const dx = x1 - x0
  const dy = y1 - y0
  const length = Math.hypot(dx, dy)
  if (length < 1.0) {
    fillDisk(grid, w, h, x0, y0, Math.max(rStart, rEnd), EMPTY)
    return
  }
  const perpX = -dy / length
  const perpY = dx / length
  const bend = rng.uniform(-wobble, wobble) * length
  const cx = (x0 + x1) / 2 + perpX * bend
  const cy = (y0 + y1) / 2 + perpY * bend

  const steps = Math.max(2, Math.trunc(length / 1.5))
  const phase = rng.uniform(0, Math.PI * 2)
  const amp = rng.uniform(0.6, 1.8)
  for (let i = 0; i <= steps; i++) {
    const t = i / steps
    const u = 1 - t
    const x = u * u * x0 + 2 * u * t * cx + t * t * x1
    const y = u * u * y0 + 2 * u * t * cy + t * t * y1
    const radius = rStart * u + rEnd * t + amp * Math.sin(phase + t * Math.PI * 2 * 1.1)
    fillDisk(grid, w, h, x, y, Math.max(3.0, radius), EMPTY)
  }
}

interface Node {
  x: number
  y: number
  r: number
}

function makeNodes(cfg: MazeConfig, rng: Rng): { nodes: Node[]; cols: number; rows: number } {
  const margin = cfg.chamberRadiusMax + 6
  const usableW = Math.max(1, cfg.width - 2 * margin)
  const usableH = Math.max(1, cfg.height - 2 * margin)
  const cols = Math.max(2, Math.round(usableW / cfg.nodeSpacing) + 1)
  const rows = Math.max(2, Math.round(usableH / cfg.nodeSpacing) + 1)
  const stepX = usableW / (cols - 1)
  const stepY = usableH / (rows - 1)
  const jitterX = Math.min(cfg.nodeJitter, stepX / 3)
  const jitterY = Math.min(cfg.nodeJitter, stepY / 3)
  const maxX = cfg.width - margin - 1
  const maxY = cfg.height - margin - 1

  const nodes: Node[] = []
  for (let j = 0; j < rows; j++) {
    for (let i = 0; i < cols; i++) {
      let x = margin + i * stepX + rng.uniform(-jitterX, jitterX)
      let y = margin + j * stepY + rng.uniform(-jitterY, jitterY)
      x = Math.min(Math.max(x, margin), maxX)
      y = Math.min(Math.max(y, margin), maxY)
      const r = rng.next() < cfg.chamberProb
        ? rng.uniform(cfg.chamberRadiusMin, cfg.chamberRadiusMax)
        : rng.uniform(cfg.corridorRadiusMin, cfg.corridorRadiusMax)
      nodes.push({ x, y, r })
    }
  }
  return { nodes, cols, rows }
}

class Dsu {
  private parent = new Map<number, number>()

  find(x: number): number {
    if (!this.parent.has(x)) this.parent.set(x, x)
    let root = x
    while (this.parent.get(root) !== root) root = this.parent.get(root) as number
    while (this.parent.get(x) !== root) {
      const next = this.parent.get(x) as number
      this.parent.set(x, root)
      x = next
    }
    return root
  }

  union(a: number, b: number): boolean {
    const ra = this.find(a)
    const rb = this.find(b)
    if (ra === rb) return false
    this.parent.set(ra, rb)
    return true
  }
}

function carveSkeleton(
  cfg: MazeConfig,
  rng: Rng,
  grid: Uint8Array,
  nodes: Node[],
  cols: number,
  rows: number,
): void {
  for (const node of nodes) {
    fillDisk(grid, cfg.width, cfg.height, node.x, node.y, node.r, EMPTY)
  }

  const edges: [number, number][] = []
  for (let j = 0; j < rows; j++) {
    for (let i = 0; i < cols; i++) {
      const idx = j * cols + i
      if (i + 1 < cols) edges.push([idx, idx + 1])
      if (j + 1 < rows) edges.push([idx, idx + cols])
      if (i + 1 < cols && j + 1 < rows) edges.push([idx, idx + cols + 1])
    }
  }
  rng.shuffle(edges)

  const dsu = new Dsu()
  for (const [a, b] of edges) {
    const isTree = dsu.union(a, b)
    if (!isTree && rng.next() >= cfg.loopProb) continue
    const na = nodes[a]
    const nb = nodes[b]
    const ra = rng.uniform(cfg.corridorRadiusMin, cfg.corridorRadiusMax)
    const rb = rng.uniform(cfg.corridorRadiusMin, cfg.corridorRadiusMax)
    carvePath(grid, cfg.width, cfg.height, [na.x, na.y], [nb.x, nb.y],
      ra, rb, rng, 0.18)
  }
}

function connectEdges(
  cfg: MazeConfig,
  rng: Rng,
  grid: Uint8Array,
  nodes: Node[],
  start: [number, number],
  end: [number, number],
): void {
  const [sx, sy] = start
  const [ex, ey] = end
  let leftIdx = 0
  let rightIdx = 0
  let leftBest = Infinity
  let rightBest = Infinity
  nodes.forEach((node, k) => {
    const dl = (node.x - sx) ** 2 + (node.y - sy) ** 2
    const dr = (node.x - ex) ** 2 + (node.y - ey) ** 2
    if (dl < leftBest) { leftBest = dl; leftIdx = k }
    if (dr < rightBest) { rightBest = dr; rightIdx = k }
  })
  const lx = nodes[leftIdx]
  const rx = nodes[rightIdx]
  const r = rng.uniform(cfg.corridorRadiusMin, cfg.corridorRadiusMax)
  carvePath(grid, cfg.width, cfg.height, [sx, sy], [lx.x, lx.y], r, r * 0.9, rng, 0.04)
  carvePath(grid, cfg.width, cfg.height, [ex, ey], [rx.x, rx.y], r, r * 0.9, rng, 0.04)
}

/** 起点→终点的 4 连通最短路径（格索引序列，含起点与终点）。 */
export function shortestPathIndices(
  grid: Uint8Array,
  w: number,
  h: number,
  start: [number, number],
  end: [number, number],
): number[] {
  const n = w * h
  const parent = new Int32Array(n).fill(-1)
  const si = start[1] * w + start[0]
  const ei = end[1] * w + end[0]
  parent[si] = si
  const queue = new Int32Array(n)
  let head = 0
  let tail = 0
  queue[tail++] = si
  while (head < tail) {
    const cur = queue[head++]
    if (cur === ei) break
    const cy = Math.floor(cur / w)
    const cx = cur - cy * w
    if (cx > 0 && parent[cur - 1] === -1 && grid[cur - 1] === EMPTY) {
      parent[cur - 1] = cur
      queue[tail++] = cur - 1
    }
    if (cx + 1 < w && parent[cur + 1] === -1 && grid[cur + 1] === EMPTY) {
      parent[cur + 1] = cur
      queue[tail++] = cur + 1
    }
    if (cy > 0 && parent[cur - w] === -1 && grid[cur - w] === EMPTY) {
      parent[cur - w] = cur
      queue[tail++] = cur - w
    }
    if (cy + 1 < h && parent[cur + w] === -1 && grid[cur + w] === EMPTY) {
      parent[cur + w] = cur
      queue[tail++] = cur + w
    }
  }
  if (parent[ei] === -1) return []
  const path: number[] = []
  let cur = ei
  while (cur !== si) {
    path.push(cur)
    cur = parent[cur]
  }
  path.push(si)
  path.reverse()
  return path
}

function buildProtected(
  path: number[],
  w: number,
  h: number,
  radius: number,
): Set<number> {
  const protectedSet = new Set<number>()
  const r2 = radius * radius
  for (let i = 0; i < path.length; i += 2) {
    const cur = path[i]
    const cy = Math.floor(cur / w)
    const cx = cur - cy * w
    const y0 = Math.max(0, Math.floor(cy - radius))
    const y1 = Math.min(h - 1, Math.ceil(cy + radius))
    for (let y = y0; y <= y1; y++) {
      const dy = y - cy
      if (dy * dy > r2) continue
      const half = Math.sqrt(r2 - dy * dy)
      const xa = Math.max(0, Math.ceil(cx - half))
      const xb = Math.min(w - 1, Math.floor(cx + half))
      for (let x = xa; x <= xb; x++) protectedSet.add(y * w + x)
    }
  }
  return protectedSet
}

function hasClearance(
  grid: Uint8Array,
  w: number,
  h: number,
  cx: number,
  cy: number,
  radius: number,
): boolean {
  for (let k = 0; k < 12; k++) {
    const a = (k * Math.PI) / 6
    const x = Math.round(cx + Math.cos(a) * radius)
    const y = Math.round(cy + Math.sin(a) * radius)
    if (!(x >= 0 && x < w && y >= 0 && y < h) || grid[y * w + x] !== EMPTY) {
      return false
    }
  }
  return true
}

function fillBlob(
  grid: Uint8Array,
  w: number,
  h: number,
  cx: number,
  cy: number,
  rx: number,
  ry: number,
  angle: number,
  protectedSet: Set<number>,
): number {
  const ca = Math.cos(angle)
  const sa = Math.sin(angle)
  const rmax = Math.max(rx, ry)
  const y0 = Math.max(0, Math.floor(cy - rmax))
  const y1 = Math.min(h - 1, Math.ceil(cy + rmax))
  let painted = 0
  for (let y = y0; y <= y1; y++) {
    const dy = y - cy
    const x0 = Math.max(0, Math.floor(cx - rmax))
    const x1 = Math.min(w - 1, Math.ceil(cx + rmax))
    for (let x = x0; x <= x1; x++) {
      const dx = x - cx
      const u = dx * ca + dy * sa
      const v = -dx * sa + dy * ca
      if ((u * u) / (rx * rx) + (v * v) / (ry * ry) <= 1.0) {
        const idx = y * w + x
        if (grid[idx] === EMPTY && !protectedSet.has(idx)) {
          grid[idx] = OBSTACLE
          painted++
        }
      }
    }
  }
  return painted
}

function addObstacles(
  cfg: MazeConfig,
  rng: Rng,
  grid: Uint8Array,
  start: [number, number],
  end: [number, number],
  protectedSet: Set<number>,
): void {
  const w = cfg.width
  const h = cfg.height
  let placed = 0
  let attempts = 0
  const maxAttempts = cfg.obstacleBlobs * 120
  while (placed < cfg.obstacleBlobs && attempts < maxAttempts) {
    attempts++
    const cx = rng.randrange(2, w - 2)
    const cy = rng.randrange(2, h - 2)
    if (grid[cy * w + cx] !== EMPTY) continue
    if (protectedSet.has(cy * w + cx)) continue
    if (Math.abs(cx - start[0]) + Math.abs(cy - start[1]) < 60) continue
    if (Math.abs(cx - end[0]) + Math.abs(cy - end[1]) < 60) continue
    const rx = rng.uniform(cfg.obstacleRadiusMin, cfg.obstacleRadiusMax)
    const ry = rng.uniform(cfg.obstacleRadiusMin, cfg.obstacleRadiusMax)
    if (!hasClearance(grid, w, h, cx, cy, Math.max(rx, ry) + 3)) continue
    const angle = rng.uniform(0, Math.PI)
    if (fillBlob(grid, w, h, cx, cy, rx, ry, angle, protectedSet)) placed++
  }
}

export class Maze {
  readonly width: number
  readonly height: number
  readonly grid: Uint8Array
  readonly start: [number, number]
  readonly end: [number, number]

  constructor(width: number, height: number, grid: Uint8Array,
              start: [number, number], end: [number, number]) {
    this.width = width
    this.height = height
    this.grid = grid
    this.start = start
    this.end = end
  }

  inBounds(x: number, y: number): boolean {
    return x >= 0 && x < this.width && y >= 0 && y < this.height
  }

  isWall(x: number, y: number): boolean {
    if (!this.inBounds(x, y)) return true
    return this.grid[y * this.width + x] === WALL
  }

  isBlocked(x: number, y: number): boolean {
    if (!this.inBounds(x, y)) return true
    return this.grid[y * this.width + x] !== EMPTY
  }

  /** 墙与通道可见；灰色障碍不可见（返回 null）。 */
  sense(x: number, y: number): number | null {
    if (!this.inBounds(x, y)) return null
    const v = this.grid[y * this.width + x]
    if (v === WALL) return WALL
    if (v === OBSTACLE) return null
    return EMPTY
  }

  isGoalMarker(x: number, y: number): boolean {
    return x === this.end[0] && y === this.end[1]
  }

  shortestPath(): [number, number][] {
    return shortestPathIndices(this.grid, this.width, this.height,
      this.start, this.end).map((idx) => {
      const y = Math.floor(idx / this.width)
      return [idx - y * this.width, y] as [number, number]
    })
  }
}

/** 生成有机迷宫（节点阵列 + Kruskal 生成树 + 圆盘笔刷 + 灰斑）。 */
export function generateMaze(cfg: MazeConfig): Maze {
  const resolved: MazeConfig = { ...DEFAULT_CONFIG, ...cfg }
  const rng = new Rng(resolved.seed >= 0 ? resolved.seed : 1)
  const w = resolved.width
  const h = resolved.height
  const grid = new Uint8Array(w * h).fill(WALL)

  const { nodes, cols, rows } = makeNodes(resolved, rng)
  const start: [number, number] = [0, Math.floor(h / 2)]
  const end: [number, number] = [w - 1, Math.floor(h / 2)]

  carveSkeleton(resolved, rng, grid, nodes, cols, rows)
  connectEdges(resolved, rng, grid, nodes, start, end)
  fillDisk(grid, w, h, start[0], start[1], 5.0, EMPTY)
  fillDisk(grid, w, h, end[0], end[1], 5.0, EMPTY)

  const path = shortestPathIndices(grid, w, h, start, end)
  const margin = resolved.obstacleRadiusMax + 3
  const protectedSet = path.length ? buildProtected(path, w, h, margin) : new Set<number>()
  addObstacles(resolved, rng, grid, start, end, protectedSet)

  grid[start[1] * w + start[0]] = EMPTY
  grid[end[1] * w + end[0]] = EMPTY
  return new Maze(w, h, grid, start, end)
}
