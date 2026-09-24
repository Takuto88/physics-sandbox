import { Vec2 } from './vec'
import { Body } from './body'
import { RNG } from './rand'
import { collidePointWithBody } from './collidePoint'

export type ParticleKind = 'sand' | 'confetti' | 'spark' | 'dust'

export interface Particle {
  x: number
  y: number
  vx: number
  vy: number
  life: number // seconds remaining; -1 = immortal
  size: number
  color: string
  kind: ParticleKind
  rot: number
  angVel: number
}

/**
 * Lightweight particle system with its own integrator (no rotation of mass,
 * no inertia bookkeeping). Sand collides with rigid bodies and with itself
 * (so it piles); confetti flutters; sparks fly and fade.
 */
export class ParticleSystem {
  particles: Particle[] = []
  gravityScale = 1
  private cellSize = 0.06
  private grid = new Map<number, number[]>()
  readonly maxCount = 4000

  constructor(readonly rng: RNG) {}

  spawn(p: Particle): void {
    if (this.particles.length < this.maxCount) this.particles.push(p)
  }

  burst(x: number, y: number, count: number, speed: number, kind: ParticleKind, color: string, spread = Math.PI * 2): void {
    for (let i = 0; i < count; i++) {
      const a = Math.random() * spread
      const s = speed * (0.3 + 0.7 * this.rng.next())
      this.spawn({
        x,
        y,
        vx: Math.cos(a) * s,
        vy: Math.sin(a) * s,
        life: kind === 'spark' ? 0.3 + this.rng.next() * 0.4 : kind === 'dust' ? 0.5 + this.rng.next() * 0.5 : -1,
        size: kind === 'sand' ? 0.02 : kind === 'spark' ? 0.012 : 0.03,
        color,
        kind,
        rot: this.rng.next() * Math.PI * 2,
        angVel: (this.rng.next() - 0.5) * 12,
      })
    }
  }

  clear(): void {
    this.particles.length = 0
  }

  step(dt: number, bodies: Body[], field: Vec2): void {
    const g = field.y * this.gravityScale
    const grid = this.grid
    grid.clear()
    const inv = 1 / this.cellSize
    const ps = this.particles

    for (let i = ps.length - 1; i >= 0; i--) {
      const p = ps[i]
      if (p.life >= 0) {
        p.life -= dt
        if (p.life <= 0) {
          ps.splice(i, 1)
          continue
        }
      }
      p.vy += g * dt
      const drag = p.kind === 'confetti' ? 1.5 : p.kind === 'spark' ? 0.5 : p.kind === 'dust' ? 2.0 : 0.1
      p.vx -= p.vx * drag * dt
      p.vy -= p.vy * drag * dt
      if (p.kind === 'confetti') {
        p.vx += Math.sin(p.rot * 3) * 0.4 * dt
      }
      p.rot += p.angVel * dt
      p.x += p.vx * dt
      p.y += p.vy * dt
    }

    // Collisions: sand and dust hit bodies; spark/confetti pass through.
    for (const p of ps) {
      if (p.kind !== 'sand' && p.kind !== 'dust') continue
      for (const b of bodies) {
        if (!isCollidable(b)) continue
        const push = collidePointWithBody(p, p.size, b)
        if (push) {
          p.x += push.x
          p.y += push.y
          // reflect the normal component with low restitution
          const vn = p.vx * push.nx + p.vy * push.ny
          if (vn < 0) {
            p.vx -= 1.5 * vn * push.nx
            p.vy -= 1.5 * vn * push.ny
          }
          p.vx *= 0.98
          p.vy *= 0.98
        }
      }
      if (p.kind === 'sand') {
        const ix = Math.floor(p.x * inv)
        const iy = Math.floor(p.y * inv)
        let cell = grid.get(ix + iy * 1_000_000)
        if (!cell) {
          cell = []
          grid.set(ix + iy * 1_000_000, cell)
        }
        cell.push(ps.indexOf(p))
      }
    }
    // sand vs sand: single relaxation pass (PBD-style push apart)
    for (const cell of grid.values()) {
      const n = cell.length
      for (let i = 0; i < n; i++) {
        for (let j = i + 1; j < n; j++) {
          const a = ps[cell[i]]
          const b2 = ps[cell[j]]
          const dx = b2.x - a.x
          const dy = b2.y - a.y
          const d2 = dx * dx + dy * dy
          const min = (a.size + b2.size) * 0.9
          if (d2 < min * min && d2 > 1e-12) {
            const d = Math.sqrt(d2)
            const corr = ((min - d) / d) * 0.5
            a.x -= dx * corr
            a.y -= dy * corr
            b2.x += dx * corr
            b2.y += dy * corr
          }
        }
      }
    }
  }
}

function isCollidable(b: Body): boolean {
  return b.kind === 'static' || (b.kind !== 'kinematic' && !b.sleeping)
}
