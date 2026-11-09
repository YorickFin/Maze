/** 浏览器版 Api：方法名与参数对齐 src/maze/visual.py，可直接顶替 window.pywebview.api。 */
import { DEFAULT_CONFIG, type MazeConfig } from './config'
import { generateMaze, Maze } from './maze'
import { randomSeed } from './rng'
import { Solver } from './solver'

export type StrategyName = 'cluster' | 'wall' | 'frontier'

function strategyName(cfg: MazeConfig): StrategyName {
  if (cfg.wallFollow) return 'wall'
  if (cfg.clusterEnabled) return 'cluster'
  return 'frontier'
}

function strategyFlags(name: StrategyName): [boolean, boolean] {
  if (name === 'wall') return [true, false]
  if (name === 'cluster') return [false, true]
  return [false, false]
}

function keyOf(idx: number, width: number): string {
  const y = Math.floor(idx / width)
  const x = idx - y * width
  return `${x},${y}`
}

export class WebApi {
  maze: Maze
  solver: Solver
  cfg: MazeConfig
  speed = 50
  strategy: StrategyName
  shortestPath: [number, number][]

  constructor() {
    this.cfg = { ...DEFAULT_CONFIG, seed: randomSeed() }
    this.maze = generateMaze(this.cfg)
    this.solver = new Solver(this.maze, this.cfg)
    this.strategy = strategyName(this.cfg)
    this.shortestPath = this.maze.shortestPath()
  }

  get_strategy(): string {
    return this.strategy
  }

  set_strategy(name: string): void {
    if (name !== 'cluster' && name !== 'wall' && name !== 'frontier') return
    this.strategy = name
    const [wallFollow, clusterEnabled] = strategyFlags(name)
    this.solver.setStrategy(wallFollow, clusterEnabled)
  }

  set_view_size(value: number): void {
    const size = Math.max(10, Math.min(100, Math.floor(Number(value))))
    this.cfg.viewSize = size
    this.solver.setViewSize(size)
  }

  get_move_mode(): string {
    return this.cfg.allowDiagonal ? '8' : '4'
  }

  set_move_mode(mode: string): void {
    const allow = String(mode) === '8'
    this.cfg.allowDiagonal = allow
    this.solver.setMoveMode(allow)
  }

  /** 不换图：回到起点，清空已探索与实时视野。 */
  reset(): void {
    this.solver = new Solver(this.maze, this.cfg)
    const [wallFollow, clusterEnabled] = strategyFlags(this.strategy)
    this.solver.setStrategy(wallFollow, clusterEnabled)
  }

  /** 换图：新随机种子重建迷宫与求解器。 */
  restart(): string {
    this.cfg.seed = randomSeed()
    this.maze = generateMaze(this.cfg)
    this.solver = new Solver(this.maze, this.cfg)
    const [wallFollow, clusterEnabled] = strategyFlags(this.strategy)
    this.solver.setStrategy(wallFollow, clusterEnabled)
    this.shortestPath = this.maze.shortestPath()
    return JSON.stringify({ seed: this.cfg.seed })
  }

  get_map(): string {
    const w = this.maze.width
    const h = this.maze.height
    const grid: number[][] = []
    for (let y = 0; y < h; y++) {
      grid.push(Array.from(this.maze.grid.subarray(y * w, (y + 1) * w)))
    }
    return JSON.stringify({
      width: w,
      height: h,
      grid,
      start: this.maze.start,
      end: this.maze.end,
      path: this.shortestPath,
      view_size: this.cfg.viewSize,
    })
  }

  get_state(): string {
    const s = this.solver
    const w = this.maze.width
    const endIdx = this.maze.end[1] * w + this.maze.end[0]
    let status: string
    if (s.pos === endIdx) status = '完成'
    else if (s.done) status = '卡死'
    else status = '探索中'

    const state = {
      pos: [s.pos % w, Math.floor(s.pos / w)],
      steps: s.stepCount,
      blocked: s.blockedHits,
      mode: s.mode,
      seed: this.cfg.seed,
      strategy: this.strategy,
      view_size: this.cfg.viewSize,
      move_mode: this.cfg.allowDiagonal ? '8' : '4',
      status,
      newExplored: s.newExplored.map((i) => keyOf(i, w)),
      newVisited: s.newVisited.map((i) => keyOf(i, w)),
      newWalls: s.newWalls.map((i) => keyOf(i, w)),
      newObstacles: s.newObstacles.map((i) => keyOf(i, w)),
    }
    s.newExplored = []
    s.newVisited = []
    s.newWalls = []
    s.newObstacles = []
    return JSON.stringify(state)
  }

  advance(): void {
    const n = 1 + this.speed * 2
    const endIdx = this.maze.end[1] * this.maze.width + this.maze.end[0]
    for (let i = 0; i < n; i++) {
      if (this.solver.pos === endIdx) break
      if (!this.solver.step()) break
    }
  }

  set_speed(value: number): void {
    this.speed = Math.floor(Number(value))
  }
}
