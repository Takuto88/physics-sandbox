import { Body } from './body'

/**
 * Broadphase: uniform spatial hash grid with an overflow list for "large"
 * bodies whose AABB would span too many cells. Cell size is fixed in world
 * meters, so cost scales with active contacts, not body count squared.
 *
 * The spec permits a grid for uniform-size scenes; at this application's
 * scale (hundreds of bodies) it is cheaper and simpler to keep correct than
 * a hand-rolled dynamic AABB tree, and it is trivially testable.
 */
export class Broadphase {
  private grid = new Map<number, Body[]>()
  private large: Body[] = []
  private pairSet = new Set<number>()
  readonly cellSize: number

  constructor(cellSize = 2) {
    this.cellSize = cellSize
  }

  private key(ix: number, iy: number): number {
    return ix + iy * 1_000_000
  }

  clear(): void {
    this.grid.clear()
    this.large.length = 0
    this.pairSet.clear()
  }

  /**
   * Collect candidate pairs (AABB overlap) and invoke `fn` for each.
   * Pairs are unique and reported once, ordered (minId, maxId) by body id.
   */
  collect(bodies: Body[], fn: (a: Body, b: Body) => void): void {
    this.clear()
    const inv = 1 / this.cellSize
    for (const b of bodies) {
      const bb = b.aabb
      const ix0 = Math.floor((bb.minX - 1e-9) * inv)
      const ix1 = Math.floor((bb.maxX + 1e-9) * inv)
      const iy0 = Math.floor((bb.minY - 1e-9) * inv)
      const iy1 = Math.floor((bb.maxY + 1e-9) * inv)
      const cells = (ix1 - ix0 + 1) * (iy1 - iy0 + 1)
      if (cells > 256 || cells < 0) {
        this.large.push(b)
        continue
      }
      for (let ix = ix0; ix <= ix1; ix++) {
        for (let iy = iy0; iy <= iy1; iy++) {
          const k = this.key(ix, iy)
          let cell = this.grid.get(k)
          if (!cell) {
            cell = []
            this.grid.set(k, cell)
          }
          cell.push(b)
        }
      }
    }

    const emit = (a: Body, b: Body) => {
      if (a.id === b.id) return
      const lo = a.id < b.id ? a : b
      const hi = a.id < b.id ? b : a
      const pk = lo.id * 1_000_000 + hi.id
      if (this.pairSet.has(pk)) return
      this.pairSet.add(pk)
      fn(lo, hi)
    }

    // grid body vs grid body, within each cell
    for (const cell of this.grid.values()) {
      const n = cell.length
      for (let i = 0; i < n; i++) {
        for (let j = i + 1; j < n; j++) {
          if (aabbOverlap(cell[i], cell[j])) emit(cell[i], cell[j])
        }
      }
    }

    // large body vs grid bodies: walk the cells the large body spans
    for (const l of this.large) {
      const bb = l.aabb
      const ix0 = Math.floor((bb.minX - 1e-9) * inv)
      const ix1 = Math.floor((bb.maxX + 1e-9) * inv)
      const iy0 = Math.floor((bb.minY - 1e-9) * inv)
      const iy1 = Math.floor((bb.maxY + 1e-9) * inv)
      for (let ix = ix0; ix <= ix1; ix++) {
        for (let iy = iy0; iy <= iy1; iy++) {
          const cell = this.grid.get(this.key(ix, iy))
          if (!cell) continue
          for (const c of cell) {
            if (aabbOverlap(l, c)) emit(l, c)
          }
        }
      }
    }

    // large vs large
    const n = this.large.length
    for (let i = 0; i < n; i++) {
      for (let j = i + 1; j < n; j++) {
        if (aabbOverlap(this.large[i], this.large[j])) emit(this.large[i], this.large[j])
      }
    }
  }
}

function aabbOverlap(a: Body, b: Body): boolean {
  const A = a.aabb
  const B = b.aabb
  return A.minX <= B.maxX && B.minX <= A.maxX && A.minY <= B.maxY && B.minY <= A.maxY
}
