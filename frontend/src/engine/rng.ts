/** 浏览器版引擎：确定性伪随机数（不追求与 Python random 同序列）。 */
export class Rng {
  private s: number

  constructor(seed: number) {
    this.s = (seed >>> 0) || 1
  }

  /** [0,1) 均匀分布（mulberry32）。 */
  next(): number {
    this.s = (this.s + 0x6d2b79f5) >>> 0
    let t = this.s
    t = Math.imul(t ^ (t >>> 15), t | 1)
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61)
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296
  }

  uniform(a: number, b: number): number {
    return a + (b - a) * this.next()
  }

  randrange(a: number, b: number): number {
    return a + Math.floor(this.next() * (b - a))
  }

  shuffle<T>(arr: T[]): void {
    for (let i = arr.length - 1; i > 0; i--) {
      const j = Math.floor(this.next() * (i + 1))
      const tmp = arr[i]
      arr[i] = arr[j]
      arr[j] = tmp
    }
  }
}

/** 生成一个随机种子（与 Python 端 Api.restart 的量级一致）。 */
export function randomSeed(): number {
  return 1 + Math.floor(Math.random() * 2147483647)
}
