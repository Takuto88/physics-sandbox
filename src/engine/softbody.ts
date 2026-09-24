import { Vec2 } from './vec'
import { Body } from './body'
import { collidePointWithBody } from './collidePoint'

/**
 * Deformable bodies via Position-Based Dynamics.
 *
 * Cloth: a grid of particles with structural distance constraints plus
 * bending constraints (i to i+2). Constraints are solved with fixed
 * position corrections; velocities are re-derived from positions, which is
 * what makes PBD unconditionally stable at any timestep.
 *
 * Soft blobs: shape matching (Macklin & Muller) — each iteration aligns the
 * particle cloud to a rigid template via a 2D polar decomposition.
 *
 * Any constraint whose stretch exceeds `breakStretch` snaps (irreversibly)
 * when breaking is enabled.
 */

export interface PBDParticle {
  x: number
  y: number
  px: number
  py: number
  vx: number
  vy: number
  invMass: number // 0 = pinned
}

export interface PBDConstraint {
  a: number
  b: number
  rest: number
  stiffness: number
  bend: boolean
  broken: boolean
}

const ITERATIONS = 6
const PARTICLE_RADIUS = 0.025
const BREAK_STRETCH = 0.35 // +35% of rest length -> snaps

export class Cloth {
  particles: PBDParticle[] = []
  constraints: PBDConstraint[] = []
  cols: number
  rows: number
  spacing: number
  color: string
  /** per-particle mass, from area density (kg/m^2) */
  particleMass: number
  /** world bodies to collide against (set by World each step) */
  private bodies: Body[] = []
  breakEnabled = true

  constructor(cols: number, rows: number, spacing: number, color = '#e8c26a', density = 0.5) {
    this.cols = cols
    this.rows = rows
    this.spacing = spacing
    this.color = color
    this.particleMass = density * spacing * spacing
    for (let r = 0; r < rows; r++) {
      for (let c = 0; c < cols; c++) {
        this.particles.push({
          x: c * spacing,
          y: r * spacing,
          px: c * spacing,
          py: r * spacing,
          vx: 0,
          vy: 0,
          invMass: 1,
        })
      }
    }
    for (let r = 0; r < rows; r++) {
      for (let c = 0; c < cols; c++) {
        const i = r * cols + c
        if (c + 1 < cols) this.addConstraint(i, i + 1, spacing, 0.9, false)
        if (r + 1 < rows) this.addConstraint(i, i + cols, spacing, 0.9, false)
        if (c + 1 < cols && r + 1 < rows) {
          this.addConstraint(i, i + cols + 1, spacing * Math.SQRT2, 0.4, false)
          this.addConstraint(i + 1, i + cols, spacing * Math.SQRT2, 0.4, false)
        }
        if (c + 2 < cols) this.addConstraint(i, i + 2, spacing * 2, 0.3, true)
        if (r + 2 < rows) this.addConstraint(i, i + cols * 2, spacing * 2, 0.3, true)
      }
    }
  }

  private addConstraint(a: number, b: number, rest: number, stiffness: number, bend: boolean): void {
    this.constraints.push({ a, b, rest, stiffness, bend, broken: false })
  }

  setBodies(bodies: Body[]): void {
    this.bodies = bodies
  }

  /** Pin particle `i` at world point (for the drape preset). */
  pin(i: number, x: number, y: number): void {
    const p = this.particles[i]
    p.x = p.px = x
    p.y = p.py = y
    p.invMass = 0
  }

  translate(dx: number, dy: number): void {
    for (const p of this.particles) {
      p.x += dx
      p.y += dy
      p.px += dx
      p.py += dy
    }
  }

  step(dt: number): void {
    const ps = this.particles
    // 1. Predict positions.
    for (const p of ps) {
      if (p.invMass === 0) continue
      p.px = p.x
      p.py = p.y
      p.x += p.vx * dt
      p.y += p.vy * dt
    }
    // 2. Solve distance constraints.
    for (let iter = 0; iter < ITERATIONS; iter++) {
      for (const c of this.constraints) {
        if (c.broken) continue
        const pa = ps[c.a]
        const pb = ps[c.b]
        const dx = pb.x - pa.x
        const dy = pb.y - pa.y
        const d = Math.hypot(dx, dy)
        if (d < 1e-9) continue
        // Breaking check.
        if (this.breakEnabled && !c.bend && (d - c.rest) / c.rest > BREAK_STRETCH) {
          c.broken = true
          continue
        }
        const wa = pa.invMass
        const wb = pb.invMass
        const wsum = wa + wb
        if (wsum === 0) continue
        const diff = ((d - c.rest) / d) * c.stiffness
        pa.x += dx * diff * (wa / wsum)
        pa.y += dy * diff * (wa / wsum)
        pb.x -= dx * diff * (wb / wsum)
        pb.y -= dy * diff * (wb / wsum)
      }
      // 3. Collide with rigid bodies.
      for (const p of ps) {
        if (p.invMass === 0) continue
        for (const b of this.bodies) {
          const push = collidePointWithBody(p, PARTICLE_RADIUS, b)
          if (push) {
            p.x += push.x
            p.y += push.y
          }
        }
      }
    }
    // 4. Re-derive velocities.
    const invDt = 1 / dt
    for (const p of ps) {
      if (p.invMass === 0) continue
      p.vx = (p.x - p.px) * invDt
      p.vy = (p.y - p.py) * invDt
    }
  }

  /** Total particle count (for HUD). */
  get count(): number {
    return this.particles.length
  }
}

// --------------------------------------------------------------- shape match

/**
 * Soft blob via shape matching: a particle cloud constrained to stay close
 * to a rigid template (a disk by default). The alignment is solved with a
 * 2D Umeyama polar decomposition each iteration.
 */
export class ShapeMatchingBlob {
  particles: PBDParticle[] = []
  private template: Vec2[] = []
  color: string
  stiffness = 0.9
  private bodies: Body[] = []
  radius: number

  constructor(radius: number, count = 40, color = '#8f7ae8') {
    this.radius = radius
    this.color = color
    // Template: points on a disk (two rings + center)
    for (let i = 0; i < count; i++) {
      const ring = i < 12 ? 0.95 : 0.55
      const a = (i % 12) * ((Math.PI * 2) / 12)
      const x = Math.cos(a) * radius * ring
      const y = Math.sin(a) * radius * ring
      this.template.push(new Vec2(x, y))
      this.particles.push({ x, y, px: x, py: y, vx: 0, vy: 0, invMass: 1 })
    }
  }

  setBodies(bodies: Body[]): void {
    this.bodies = bodies
  }

  translate(dx: number, dy: number): void {
    for (const p of this.particles) {
      p.x += dx
      p.y += dy
      p.px += dx
      p.py += dy
    }
  }

  step(dt: number): void {
    const ps = this.particles
    for (const p of ps) {
      p.px = p.x
      p.py = p.y
      p.x += p.vx * dt
      p.y += p.vy * dt
    }
    for (let iter = 0; iter < 4; iter++) {
      this.shapeMatch()
      for (const p of ps) {
        for (const b of this.bodies) {
          const push = collidePointWithBody(p, this.radius / 6, b)
          if (push) {
            p.x += push.x
            p.y += push.y
          }
        }
      }
    }
    const invDt = 1 / dt
    for (const p of ps) {
      p.vx = (p.x - p.px) * invDt
      p.vy = (p.y - p.py) * invDt
    }
  }

  private shapeMatch(): void {
    const t = this.template
    const ps = this.particles
    const n = t.length
    // Current centroid.
    let cx = 0
    let cy = 0
    for (const p of ps) {
      cx += p.x
      cy += p.y
    }
    cx /= n
    cy /= n
    // Template centroid (points are near origin; compute anyway).
    let tx = 0
    let ty = 0
    for (const q of t) {
      tx += q.x
      ty += q.y
    }
    tx /= n
    ty /= n
    // Covariance S = sum (x_i - cx)(t_i - tx)^T
    let sxx = 0
    let sxy = 0
    let syx = 0
    let syy = 0
    for (let i = 0; i < n; i++) {
      const dx = ps[i].x - cx
      const dy = ps[i].y - cy
      const ex = t[i].x - tx
      const ey = t[i].y - ty
      sxx += dx * ex
      sxy += dx * ey
      syx += dy * ex
      syy += dy * ey
    }
    // 2D Umeyama: find rotation aligning template to current cloud.
    const trace = sxx + syy
    const det = sxx * syy - sxy * syx
    const r = Math.sqrt(Math.max(0, (trace + Math.sqrt(trace * trace - 4 * det * det)) / 2))
    let rotX = 0
    let rotY = 0
    if (r > 1e-12) {
      rotX = (sxx + r) / (2 * r)
      rotY = sxy / (2 * r)
    } else {
      rotX = 1
    }
    // Normalize (numerical safety).
    const rl = Math.hypot(rotX, rotY) || 1
    rotX /= rl
    rotY /= rl
    // Target transform: q_i = R * (t_i - tx) + (cx - R * tx)
    for (let i = 0; i < n; i++) {
      const ex = t[i].x - tx
      const ey = t[i].y - ty
      const qx = rotX * ex - rotY * ey + cx - (rotX * tx - rotY * ty)
      const qy = rotX * ey + rotY * ex + cy - (rotX * tx + rotY * ty)
      ps[i].x += this.stiffness * (qx - ps[i].x)
      ps[i].y += this.stiffness * (qy - ps[i].y)
    }
  }
}
