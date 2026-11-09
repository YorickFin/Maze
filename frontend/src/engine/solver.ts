/** 浏览器版求解器：沿墙 / 前沿规划 / 聚类信息增益（与 src/maze/solver.py 同策略）。 */
import { EMPTY, Maze, WALL } from './maze'
import type { MazeConfig } from './config'

const KNOWN_EMPTY = 1
const KNOWN_WALL = 2
const KNOWN_OBSTACLE = 3

export const MODE_WALL = '沿墙'
export const MODE_PLAN = '规划'

const DIRS4: [number, number][] = [[1, 0], [0, 1], [0, -1], [-1, 0]]
const DIRS8: [number, number][] = [
  [1, 0], [1, 1], [0, 1], [-1, 1], [-1, 0], [-1, -1], [0, -1], [1, -1],
]
const ENTRY_DIRS: [number, number][] = [
  [1, 0], [0, 1], [0, -1], [-1, 0], [1, 1], [1, -1], [-1, 1], [-1, -1],
]
const WALL_DIRS4: [number, number][] = [[1, 0], [0, 1], [-1, 0], [0, -1]]
const WALL_TURNS4 = [1, 0, 3, 2]
const WALL_TURNS8 = [2, 1, 0, 7, 6, 5, 3, 4]
const WALL_DRIFT8 = [0, 1, 7, 2, 6, 5, 3, 4]

interface HeapItem {
  cost: number
  counter: number
  idx: number
}

class MinHeap {
  private items: HeapItem[] = []

  get size(): number {
    return this.items.length
  }

  push(cost: number, counter: number, idx: number): void {
    const item = { cost, counter, idx }
    this.items.push(item)
    let i = this.items.length - 1
    while (i > 0) {
      const p = (i - 1) >> 1
      if (this.less(this.items[i], this.items[p])) {
        const tmp = this.items[i]
        this.items[i] = this.items[p]
        this.items[p] = tmp
        i = p
      } else break
    }
  }

  pop(): HeapItem | undefined {
    if (!this.items.length) return undefined
    const top = this.items[0]
    const last = this.items.pop() as HeapItem
    if (this.items.length) {
      this.items[0] = last
      let i = 0
      while (true) {
        const l = i * 2 + 1
        const r = l + 1
        let m = i
        if (l < this.items.length && this.less(this.items[l], this.items[m])) m = l
        if (r < this.items.length && this.less(this.items[r], this.items[m])) m = r
        if (m === i) break
        const tmp = this.items[i]
        this.items[i] = this.items[m]
        this.items[m] = tmp
        i = m
      }
    }
    return top
  }

  private less(a: HeapItem, b: HeapItem): boolean {
    return a.cost < b.cost || (a.cost === b.cost && a.counter < b.counter)
  }
}

export class Solver {
  readonly maze: Maze
  viewSize: number
  readonly maxSteps: number
  private readonly costRight: number
  private readonly costVertical: number
  private readonly costLeft: number
  private readonly suspectPenalty: number
  private readonly planExpandLimit: number
  wallFollow: boolean
  clusterEnabled: boolean
  private readonly clusterGainWeight: number
  private readonly clusterRightWeight: number
  private readonly clusterCostWeight: number
  private readonly clusterMinSize: number
  clusterTopK: number
  private readonly clusterGainRadius: number
  clusterExpandLimit: number
  allowDiagonal: boolean
  private readonly diagonalFactor: number

  private planDirs: [number, number][]
  private wallDirs: [number, number][]
  private wallTurns: number[]
  private entryDirs: [number, number][]

  mode: string
  private facing = 0
  private wallTurn = 0
  private wallStates: Set<string>
  wallFailed = false

  private known = new Map<number, number>()
  pos: number
  visited: number[] = []
  private visitedSet = new Set<number>()
  explored = new Set<number>()
  goal: number | null = null

  stepCount = 0
  blockedHits = 0
  done = false

  private path: number[] = []
  private pathIdx = 0
  private entry: number | null = null

  newExplored: number[] = []
  newVisited: number[] = []
  newWalls: number[] = []
  newObstacles: number[] = []

  private reach = new Set<number>()
  private frontier = new Set<number>()
  private obsPos: number | null = null

  constructor(maze: Maze, cfg: MazeConfig) {
    this.maze = maze
    this.viewSize = cfg.viewSize
    this.maxSteps = cfg.maxSteps
    this.costRight = cfg.costRight
    this.costVertical = cfg.costVertical
    this.costLeft = cfg.costLeft
    this.suspectPenalty = cfg.suspectPenalty
    this.planExpandLimit = cfg.planExpandLimit
    this.wallFollow = cfg.wallFollow
    this.clusterEnabled = cfg.clusterEnabled
    this.clusterGainWeight = cfg.clusterGainWeight
    this.clusterRightWeight = cfg.clusterRightWeight
    this.clusterCostWeight = cfg.clusterCostWeight
    this.clusterMinSize = cfg.clusterMinSize
    this.clusterTopK = cfg.clusterTopK
    this.clusterGainRadius = cfg.clusterGainRadius
    this.clusterExpandLimit = cfg.clusterExpandLimit
    this.allowDiagonal = cfg.allowDiagonal
    this.diagonalFactor = cfg.diagonalFactor

    if (cfg.allowDiagonal) {
      this.clusterTopK = Math.min(this.clusterTopK, 2)
      this.clusterExpandLimit = Math.min(this.clusterExpandLimit, 3000)
    }
    this.planDirs = cfg.allowDiagonal ? DIRS8 : DIRS4
    this.wallDirs = cfg.allowDiagonal ? DIRS8 : WALL_DIRS4
    this.wallTurns = cfg.allowDiagonal ? WALL_TURNS8 : WALL_TURNS4
    this.entryDirs = cfg.allowDiagonal ? ENTRY_DIRS : DIRS4

    this.mode = cfg.wallFollow ? MODE_WALL : MODE_PLAN
    this.pos = maze.start[1] * maze.width + maze.start[0]
    this.visited = [this.pos]
    this.visitedSet.add(this.pos)
    this.wallStates = new Set<string>([this.stateKey(this.pos, 0)])
    this.known.set(this.pos, KNOWN_EMPTY)
    this.reach.add(this.pos)
    this.observe(true)
  }

  private stateKey(pos: number, facing: number): string {
    return `${pos}:${facing}`
  }

  private idx(x: number, y: number): number {
    return y * this.maze.width + x
  }

  private xOf(idx: number): number {
    return idx % this.maze.width
  }

  private yOf(idx: number): number {
    return Math.floor(idx / this.maze.width)
  }

  setStrategy(wallFollow: boolean, clusterEnabled: boolean): void {
    this.wallFollow = wallFollow
    this.clusterEnabled = clusterEnabled
    this.clearPlan()
    if (wallFollow) {
      this.mode = MODE_WALL
      this.facing = 0
      this.wallTurn = 0
      this.wallFailed = false
      this.wallStates = new Set<string>([this.stateKey(this.pos, 0)])
    } else {
      this.mode = MODE_PLAN
    }
  }

  setViewSize(value: number): void {
    const size = Math.max(1, Math.floor(value))
    if (size === this.viewSize) return
    this.viewSize = size
    this.obsPos = null
    this.observe(true)
  }

  setMoveMode(allowDiagonal: boolean): void {
    this.allowDiagonal = allowDiagonal
    this.planDirs = allowDiagonal ? DIRS8 : DIRS4
    this.wallDirs = allowDiagonal ? DIRS8 : WALL_DIRS4
    this.wallTurns = allowDiagonal ? WALL_TURNS8 : WALL_TURNS4
    this.entryDirs = allowDiagonal ? ENTRY_DIRS : DIRS4
    this.clearPlan()
    this.facing = 0
    this.wallTurn = 0
    this.wallFailed = false
    this.wallStates = new Set<string>([this.stateKey(this.pos, 0)])
    this.mode = this.wallFollow ? MODE_WALL : MODE_PLAN
  }

  // ---- 感知 ----

  private observe(full = false): void {
    const half = Math.floor(this.viewSize / 2)
    const n0 = this.newExplored.length
    const cells: number[] = []
    const w = this.maze.width
    const h = this.maze.height
    const px = this.xOf(this.pos)
    const py = this.yOf(this.pos)

    if (full || this.obsPos === null) {
      const x0 = px - half
      const y0 = py - half
      for (let j = 0; j < this.viewSize; j++) {
        for (let i = 0; i < this.viewSize; i++) {
          const gx = x0 + i
          const gy = y0 + j
          if (gx >= 0 && gx < w && gy >= 0 && gy < h) cells.push(this.idx(gx, gy))
        }
      }
    } else {
      const ox = this.xOf(this.obsPos)
      const oy = this.yOf(this.obsPos)
      if (px === ox && py === oy) return
      // 视野范围 [pos - half, pos - half + viewSize - 1]；偶数视野时右/下边界不是 pos + half
      const left = px - half
      const right = px - half + this.viewSize - 1
      const top = py - half
      const bottom = py - half + this.viewSize - 1
      if (px !== ox) {
        const x = px > ox ? right : left
        for (let y = top; y <= bottom; y++) {
          if (x >= 0 && x < w && y >= 0 && y < h) cells.push(this.idx(x, y))
        }
      }
      if (py !== oy) {
        const y = py > oy ? bottom : top
        for (let x = left; x <= right; x++) {
          if (x >= 0 && x < w && y >= 0 && y < h) cells.push(this.idx(x, y))
        }
      }
    }

    for (const cell of cells) {
      const gx = this.xOf(cell)
      const gy = this.yOf(cell)
      if (this.goal === null && this.maze.isGoalMarker(gx, gy)) this.goal = cell
      if (!this.explored.has(cell)) {
        this.explored.add(cell)
        this.newExplored.push(cell)
      }
      if (this.known.has(cell)) continue
      const v = this.maze.sense(gx, gy)
      if (v === null) continue
      if (v === WALL) {
        this.known.set(cell, KNOWN_WALL)
        this.newWalls.push(cell)
      } else if (v === EMPTY) {
        this.known.set(cell, KNOWN_EMPTY)
        this.growReach(cell)
      }
    }
    this.obsPos = this.pos

    const fresh = this.newExplored.slice(n0)
    for (const cell of fresh) {
      this.updateFrontier(cell)
      const x = this.xOf(cell)
      const y = this.yOf(cell)
      for (const [dx, dy] of DIRS8) {
        const nx = x + dx
        const ny = y + dy
        if (nx >= 0 && nx < w && ny >= 0 && ny < h) this.updateFrontier(this.idx(nx, ny))
      }
    }
  }

  private updateFrontier(cell: number): void {
    if (this.known.get(cell) === KNOWN_EMPTY && this.unexploredNeighbor(cell) !== null) {
      this.frontier.add(cell)
    } else {
      this.frontier.delete(cell)
    }
  }

  private markBlocked(x: number, y: number): void {
    const cell = this.idx(x, y)
    if (this.maze.isWall(x, y)) {
      this.known.set(cell, KNOWN_WALL)
      this.newWalls.push(cell)
    } else {
      this.known.set(cell, KNOWN_OBSTACLE)
      this.newObstacles.push(cell)
    }
    this.blockedHits++
  }

  private growReach(seed: number): void {
    if (this.reach.has(seed) || this.known.get(seed) !== KNOWN_EMPTY) return
    const x = this.xOf(seed)
    const y = this.yOf(seed)
    let connected = false
    for (const [dx, dy] of DIRS4) {
      if (this.reach.has(this.idx(x + dx, y + dy))) {
        connected = true
        break
      }
    }
    if (!connected) return
    const stack = [seed]
    this.reach.add(seed)
    while (stack.length) {
      const cur = stack.pop() as number
      const cx = this.xOf(cur)
      const cy = this.yOf(cur)
      for (const [dx, dy] of DIRS4) {
        const nb = this.idx(cx + dx, cy + dy)
        if (!this.reach.has(nb) && this.known.get(nb) === KNOWN_EMPTY) {
          this.reach.add(nb)
          stack.push(nb)
        }
      }
    }
  }

  private tryMove(nx: number, ny: number, allowBrush = false): boolean {
    const px = this.xOf(this.pos)
    const py = this.yOf(this.pos)
    const dx = nx - px
    const dy = ny - py
    if (dx && dy && !this.allowDiagonal) return false
    if (this.maze.isBlocked(nx, ny)) {
      this.markBlocked(nx, ny)
      return false
    }
    if (dx && dy && !allowBrush) {
      for (const [sx, sy] of [[nx, py], [px, ny]]) {
        if (this.maze.isBlocked(sx, sy)) {
          this.markBlocked(sx, sy)
          return false
        }
      }
    }
    const cell = this.idx(nx, ny)
    this.known.set(cell, KNOWN_EMPTY)
    this.growReach(cell)
    this.pos = cell
    if (!this.visitedSet.has(cell)) {
      this.visitedSet.add(cell)
      this.visited.push(cell)
      this.newVisited.push(cell)
    }
    this.observe()
    return true
  }

  // ---- 规划 ----

  private directionCost(dx: number, dy = 0): number {
    let base: number
    if (dx > 0) base = this.costRight
    else if (dx < 0) base = this.costLeft
    else base = this.costVertical
    if (dx && dy) base *= this.diagonalFactor
    return base
  }

  private unexploredNeighbor(cell: number): number | null {
    const x = this.xOf(cell)
    const y = this.yOf(cell)
    for (const [dx, dy] of this.entryDirs) {
      const nx = x + dx
      const ny = y + dy
      if (!this.maze.inBounds(nx, ny)) continue
      const nb = this.idx(nx, ny)
      if (this.known.has(nb) || this.explored.has(nb)) continue
      if (dx && dy) {
        if (this.known.get(this.idx(nx, y)) !== KNOWN_EMPTY
          || this.known.get(this.idx(x, ny)) !== KNOWN_EMPTY) continue
      }
      return nb
    }
    return null
  }

  private planToGoal(): boolean {
    const goal = this.goal
    if (goal === null || this.known.get(goal) !== KNOWN_EMPTY || !this.reach.has(goal)) {
      return false
    }
    if (this.pos === goal) {
      this.path = [goal]
      this.pathIdx = 0
      this.entry = null
      return true
    }
    const heap = new MinHeap()
    heap.push(0, 0, this.pos)
    const gScore = new Map<number, number>([[this.pos, 0]])
    const cameFrom = new Map<number, number>()
    let counter = 1
    let expanded = 0
    while (heap.size) {
      const item = heap.pop() as HeapItem
      if (item.cost > (gScore.get(item.idx) ?? Infinity)) continue
      if (item.idx === goal) {
        const path = [item.idx]
        let cur = item.idx
        while (cameFrom.has(cur)) {
          cur = cameFrom.get(cur) as number
          path.push(cur)
        }
        path.reverse()
        this.path = path
        this.pathIdx = 0
        this.entry = null
        return true
      }
      expanded++
      if (expanded > this.planExpandLimit) break
      const cx = this.xOf(item.idx)
      const cy = this.yOf(item.idx)
      for (const [dx, dy] of this.planDirs) {
        const nx = cx + dx
        const ny = cy + dy
        if (!this.maze.inBounds(nx, ny)) continue
        const nb = this.idx(nx, ny)
        if (this.known.get(nb) !== KNOWN_EMPTY) continue
        if (dx && dy) {
          if (this.known.get(this.idx(nx, cy)) !== KNOWN_EMPTY
            || this.known.get(this.idx(cx, ny)) !== KNOWN_EMPTY) continue
        }
        const nd = item.cost + this.directionCost(dx, dy)
        if (nd < (gScore.get(nb) ?? Infinity)) {
          gScore.set(nb, nd)
          cameFrom.set(nb, item.idx)
          counter++
          heap.push(nd, counter, nb)
        }
      }
    }
    return false
  }

  private frontierClusters(): number[][] {
    const seen = new Set<number>()
    const clusters: number[][] = []
    for (const start of this.frontier) {
      if (seen.has(start)) continue
      const comp: number[] = []
      const stack = [start]
      seen.add(start)
      while (stack.length) {
        const q = stack.pop() as number
        comp.push(q)
        const qx = this.xOf(q)
        const qy = this.yOf(q)
        for (const [dx, dy] of DIRS4) {
          const nb = this.idx(qx + dx, qy + dy)
          if (this.frontier.has(nb) && !seen.has(nb)) {
            seen.add(nb)
            stack.push(nb)
          }
        }
      }
      if (comp.length >= this.clusterMinSize) clusters.push(comp)
    }
    return clusters
  }

  private gain(cell: number, radius: number): number {
    const cx = this.xOf(cell)
    const cy = this.yOf(cell)
    let cnt = 0
    for (let y = cy - radius; y <= cy + radius; y++) {
      for (let x = cx - radius; x <= cx + radius; x++) {
        if (this.maze.inBounds(x, y) && !this.explored.has(this.idx(x, y))) cnt++
      }
    }
    return cnt
  }

  private planCluster(): boolean {
    const clusters = this.frontierClusters()
    if (!clusters.length) return false
    const startY = this.yOf(this.pos)
    const reps: number[] = clusters.map((comp) => {
      let best = comp[0]
      let bestX = this.xOf(best)
      let bestScore = -Math.abs(this.yOf(best) - startY)
      for (const p of comp) {
        const px = this.xOf(p)
        const score = -Math.abs(this.yOf(p) - startY)
        if (px > bestX || (px === bestX && score > bestScore)) {
          best = p
          bestX = px
          bestScore = score
        }
      }
      return best
    })

    const heap = new MinHeap()
    heap.push(0, 0, this.pos)
    const gScore = new Map<number, number>([[this.pos, 0]])
    const cameFrom = new Map<number, number>()
    let counter = 1
    let expanded = 0
    const remaining = new Set<number>(reps)
    const found: { cost: number; idx: number }[] = []

    while (heap.size && found.length < this.clusterTopK) {
      const item = heap.pop() as HeapItem
      if (item.cost > (gScore.get(item.idx) ?? Infinity)) continue
      if (remaining.has(item.idx)) {
        found.push({ cost: item.cost, idx: item.idx })
        remaining.delete(item.idx)
      }
      expanded++
      if (expanded > this.clusterExpandLimit) break
      const cx = this.xOf(item.idx)
      const cy = this.yOf(item.idx)
      for (const [dx, dy] of this.planDirs) {
        const nx = cx + dx
        const ny = cy + dy
        if (!this.maze.inBounds(nx, ny)) continue
        const nb = this.idx(nx, ny)
        if (this.known.get(nb) !== KNOWN_EMPTY) continue
        if (dx && dy) {
          if (this.known.get(this.idx(nx, cy)) !== KNOWN_EMPTY
            || this.known.get(this.idx(cx, ny)) !== KNOWN_EMPTY) continue
        }
        const nd = item.cost + this.directionCost(dx, dy)
        if (nd < (gScore.get(nb) ?? Infinity)) {
          gScore.set(nb, nd)
          cameFrom.set(nb, item.idx)
          counter++
          heap.push(nd, counter, nb)
        }
      }
    }

    if (!found.length) return false

    let best = found[0]
    let bestUtility = -Infinity
    for (const item of found) {
      const gain = this.gain(item.idx, this.clusterGainRadius)
      const dx = this.xOf(item.idx) - this.xOf(this.pos)
      const utility = this.clusterGainWeight * gain
        + this.clusterRightWeight * dx
        - this.clusterCostWeight * item.cost
      if (utility > bestUtility) {
        bestUtility = utility
        best = item
      }
    }

    const entry = this.unexploredNeighbor(best.idx)
    if (entry === null) return false
    const path = [best.idx]
    let cur = best.idx
    while (cameFrom.has(cur)) {
      cur = cameFrom.get(cur) as number
      path.push(cur)
    }
    path.reverse()
    this.path = path
    this.pathIdx = 0
    this.entry = entry
    return true
  }

  private plan(): boolean {
    if (this.planToGoal()) return true
    if (this.clusterEnabled && this.planCluster()) return true

    const heap = new MinHeap()
    heap.push(0, 0, this.pos)
    const gScore = new Map<number, number>([[this.pos, 0]])
    const cameFrom = new Map<number, number>()
    let counter = 1
    let expanded = 0
    let bestCell: number | null = null
    let bestKeyX = -Infinity
    let bestKeyY = -Infinity

    while (heap.size) {
      const item = heap.pop() as HeapItem
      if (item.cost > (gScore.get(item.idx) ?? Infinity)) continue
      if (this.known.get(item.idx) === KNOWN_EMPTY) {
        const entry = this.unexploredNeighbor(item.idx)
        if (entry !== null) {
          const path = [item.idx]
          let cur = item.idx
          while (cameFrom.has(cur)) {
            cur = cameFrom.get(cur) as number
            path.push(cur)
          }
          path.reverse()
          this.path = path
          this.pathIdx = 0
          this.entry = entry
          return true
        }
        const keyX = this.xOf(item.idx)
        const keyY = -Math.abs(this.yOf(item.idx) - this.yOf(this.pos))
        if (keyX > bestKeyX || (keyX === bestKeyX && keyY > bestKeyY)) {
          bestKeyX = keyX
          bestKeyY = keyY
          bestCell = item.idx
        }
      }

      expanded++
      if (expanded > this.planExpandLimit) break

      const cx = this.xOf(item.idx)
      const cy = this.yOf(item.idx)
      for (const [dx, dy] of this.planDirs) {
        const nx = cx + dx
        const ny = cy + dy
        if (!this.maze.inBounds(nx, ny)) continue
        const nb = this.idx(nx, ny)
        const state = this.known.get(nb)
        if (state === KNOWN_WALL || state === KNOWN_OBSTACLE) continue
        let stepCost: number
        if (state === undefined) {
          if (dx && dy) continue
          if (!this.explored.has(nb)) continue
          stepCost = this.directionCost(dx, dy) + this.suspectPenalty
        } else {
          if (dx && dy) {
            if (this.known.get(this.idx(nx, cy)) !== KNOWN_EMPTY
              || this.known.get(this.idx(cx, ny)) !== KNOWN_EMPTY) continue
          }
          stepCost = this.directionCost(dx, dy)
        }
        const nd = item.cost + stepCost
        if (nd < (gScore.get(nb) ?? Infinity)) {
          gScore.set(nb, nd)
          cameFrom.set(nb, item.idx)
          counter++
          heap.push(nd, counter, nb)
        }
      }
    }

    if (bestCell !== null && bestCell !== this.pos) {
      const path = [bestCell]
      let cur = bestCell
      while (cameFrom.has(cur)) {
        cur = cameFrom.get(cur) as number
        path.push(cur)
      }
      path.reverse()
      this.path = path
      this.pathIdx = 0
      this.entry = null
      return true
    }
    return false
  }

  private clearPlan(): void {
    this.path = []
    this.pathIdx = 0
    this.entry = null
  }

  // ---- 沿墙 ----

  private knownBlocked(x: number, y: number): boolean {
    if (!this.maze.inBounds(x, y)) return true
    const state = this.known.get(this.idx(x, y))
    return state === KNOWN_WALL || state === KNOWN_OBSTACLE
  }

  private wallStep(): boolean {
    if (this.allowDiagonal) return this.wallStep8()
    const px = this.xOf(this.pos)
    const py = this.yOf(this.pos)
    for (let k = this.wallTurn; k < this.wallTurns.length; k++) {
      const nd = (this.facing + this.wallTurns[k]) % this.wallDirs.length
      const [dx, dy] = this.wallDirs[nd]
      const nx = px + dx
      const ny = py + dy
      if (!this.maze.inBounds(nx, ny)) {
        this.wallTurn = k + 1
        continue
      }
      const state = this.known.get(this.idx(nx, ny))
      if (state === KNOWN_WALL || state === KNOWN_OBSTACLE) {
        this.wallTurn = k + 1
        continue
      }
      if (dx && dy) {
        const s1 = this.known.get(this.idx(nx, py))
        const s2 = this.known.get(this.idx(px, ny))
        if (s1 === KNOWN_WALL || s1 === KNOWN_OBSTACLE
          || s2 === KNOWN_WALL || s2 === KNOWN_OBSTACLE) {
          this.wallTurn = k + 1
          continue
        }
      }
      this.wallTurn = k
      if (this.tryMove(nx, ny)) {
        this.facing = nd
        this.wallTurn = 0
        this.recordWallState()
      } else {
        this.wallTurn = k + 1
      }
      return true
    }
    this.wallFailed = true
    return false
  }

  private wallStep8(): boolean {
    const px = this.xOf(this.pos)
    const py = this.yOf(this.pos)
    const n = this.wallDirs.length
    for (const contactOnly of [true, false]) {
      const seq = contactOnly ? this.wallTurns : WALL_DRIFT8
      for (const off of seq) {
        const nd = (this.facing + off) % n
        const [dx, dy] = this.wallDirs[nd]
        const nx = px + dx
        const ny = py + dy
        if (this.knownBlocked(nx, ny)) continue
        if (dx && dy) {
          let sides = 0
          if (this.knownBlocked(nx, py)) sides++
          if (this.knownBlocked(px, ny)) sides++
          if (sides > 1) continue
        }
        if (contactOnly && !this.keepsWallOnRight(nx, ny, nd)) continue
        if (this.tryMove(nx, ny, !!(dx && dy))) {
          this.facing = nd
          this.wallTurn = 0
          this.recordWallState()
          return true
        }
      }
    }
    this.wallFailed = true
    return false
  }

  private keepsWallOnRight(x: number, y: number, nd: number): boolean {
    const n = this.wallDirs.length
    for (const k of [1, 2, 3]) {
      const d = (nd + k) % n
      const [dx, dy] = this.wallDirs[d]
      if (this.knownBlocked(x + dx, y + dy)) return true
    }
    return false
  }

  private recordWallState(): void {
    const key = this.stateKey(this.pos, this.facing)
    if (this.wallStates.has(key)) {
      this.wallFailed = true
      this.mode = MODE_PLAN
    } else {
      this.wallStates.add(key)
    }
  }

  // ---- 主步进 ----

  step(): boolean {
    if (this.done) return false
    if (this.goal !== null) {
      if (this.pos === this.goal) {
        this.done = true
        return false
      }
      this.mode = MODE_PLAN
    }
    if (this.stepCount >= this.maxSteps) {
      this.done = true
      return false
    }
    this.stepCount++
    if (this.mode === MODE_WALL) {
      if (this.wallStep()) return true
      this.mode = MODE_PLAN
    }
    return this.plannerStep()
  }

  private plannerStep(): boolean {
    if (!this.path.length) {
      if (!this.plan()) {
        this.done = true
        return false
      }
    }
    if (this.pathIdx + 1 < this.path.length) {
      const nxt = this.path[this.pathIdx + 1]
      this.pathIdx++
      const nx = this.xOf(nxt)
      const ny = this.yOf(nxt)
      if (!this.tryMove(nx, ny)) this.clearPlan()
      return true
    }
    const entry = this.entry
    this.clearPlan()
    if (entry === null) return true
    this.tryMove(this.xOf(entry), this.yOf(entry))
    return true
  }
}
