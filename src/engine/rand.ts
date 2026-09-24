/**
 * Deterministic PRNG (mulberry32). The world owns a single instance so a
 * saved scene replays identically: same seed -> same sequence.
 */
export class RNG {
  private s: number

  constructor(seed: number) {
    this.s = seed >>> 0
    if (this.s === 0) this.s = 0x9e3779b9
  }

  /** Uniform float in [0, 1). */
  next(): number {
    this.s |= 0
    this.s = (this.s + 0x6d2b79f5) | 0
    let t = Math.imul(this.s ^ (this.s >>> 15), 1 | this.s)
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296
  }

  /** Uniform float in [min, max). */
  range(min: number, max: number): number {
    return min + (max - min) * this.next()
  }

  get seed(): number {
    return this.s >>> 0
  }
}
