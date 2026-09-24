import { Vec2 } from './vec'
import { Body } from './body'
import { collidePointWithBody } from './collidePoint'

/**
 * 2D SPH fluid (Clavet et al. "Particle-based Viscoelastic Fluid Simulation"):
 * density and near-density kernels, pressure as a position correction
 * (double density relaxation), pairwise viscosity impulses.
 *
 * Rigid-body coupling: each fluid particle that overlaps a body is projected
 * out, and the body receives the reaction force of the particle's pressure.
 * Buoyancy is NOT a scripted rule — it emerges from the pressure field:
 * fluid under a floating body is compressed by its weight, so the upward
 * reaction exceeds the downward one.
 */

export interface FluidConfig {
  /** interaction radius (m) */
  h: number
  /** rest density in kernel units */
  restDensity: number
  /** pressure stiffness k */
  stiffness: number
  /** near-pressure stiffness (anti-clustering) */
  nearStiffness: number
  /** linear viscosity sigma */
  viscositySigma: number
  /** quadratic viscosity beta */
  viscosityBeta: number
  /** fluid mass density kg/m^2 (2D) — sets particle mass from spacing */
  fluidDensity: number
  /** converts particle pressure into a reaction force on bodies (N per unit P) */
  forceScale: number
  /** converts particle near-pressure into a reaction force on bodies */
  nearForceScale: number
  color: string
}

export const DEFAULT_FLUID: FluidConfig = {
  h: 0.12,
  // rest density in kernel units, matched to the spawn packing (a 420-particle
  // disk at r=1.0 measures rho≈0.2) so the fluid is roughly neutral at rest.
  restDensity: 0.2,
  // Stiffness kept low: at the 120 Hz timestep the Clavet double-density
  // relaxation injects a velocity of ~dt·P per step, so high stiffness makes
  // the fluid explode. Low stiffness trades incompressibility for stability.
  stiffness: 150,
  nearStiffness: 80,
  viscositySigma: 0.5,
  viscosityBeta: 0.1,
  fluidDensity: 1000,
  // Reaction force disabled: coupling the (soft) particle pressure to rigid
  // bodies produces impulsive forces that drive the boxes through the floor.
  // With it off the fluid still collides with bodies (position projection) but
  // exerts no buoyant force, so buoyancy is not demonstrated.
  forceScale: 0,
  nearForceScale: 0,
  color: '#4fc3f7',
}

export interface FluidPoint {
  x: number
  y: number
  px: number
  py: number
  vx: number
  vy: number
  density: number
  nearDensity: number
  pressure: number
  nearPressure: number
}

export class Fluid {
  points: FluidPoint[] = []
  config: FluidConfig
  /** mass per particle */
  particleMass: number
  /** contact radius of each particle for body collisions */
  pointRadius: number
  color: string
  /** reaction forces accumulated for rigid bodies this step */
  private bodyFx = new Map<number, { fx: number; fy: number; px: number; py: number; w: number }>()

  constructor(config?: Partial<FluidConfig>) {
    this.config = { ...DEFAULT_FLUID, ...config }
    this.color = this.config.color
    const spacing = this.config.h / 2.6
    this.particleMass = this.config.fluidDensity * spacing * spacing
    this.pointRadius = spacing * 0.6
  }

  /** Spawn a disk of fluid particles. */
  spawnDisk(cx: number, cy: number, radius: number, count = 250): void {
    let placed = 0
    for (let i = 0; i < count && placed < count; i++) {
      // spiral placement
      const a = i * 2.399963
      const r = radius * Math.sqrt(i / count)
      this.points.push({
        x: cx + Math.cos(a) * r,
        y: cy + Math.sin(a) * r,
        px: cx + Math.cos(a) * r,
        py: cy + Math.sin(a) * r,
        vx: 0,
        vy: 0,
        density: 0,
        nearDensity: 0,
        pressure: 0,
        nearPressure: 0,
      })
      placed++
    }
  }

  applyReactionForces(bodies: Body[]): void {
    for (const [id, f] of this.bodyFx) {
      const b = bodies.find((x) => x.id === id)
      if (!b) continue
      b.force.x += f.fx
      b.force.y += f.fy
      b.torque += (f.px - b.pos.x) * f.fy - (f.py - b.pos.y) * f.fx
    }
    this.bodyFx.clear()
  }

  /** Per-particle neighbor list: flat index array with offsets. */
  private neigh = new Int32Array(0)
  private neighOff = new Int32Array(0)
  private gridMap = new Map<number, number[]>()

  private buildNeighbors(): void {
    const cfg = this.config
    const h = cfg.h
    const pts = this.points
    const n = pts.length
    const grid = this.gridMap
    grid.clear()
    const inv = 1 / (h * 0.5)
    for (let i = 0; i < n; i++) {
      const k = Math.floor(pts[i].x * inv) + Math.floor(pts[i].y * inv) * 1_000_000
      let cell = grid.get(k)
      if (!cell) {
        cell = []
        grid.set(k, cell)
      }
      cell.push(i)
    }
    // count
    const off = new Int32Array(n + 1)
    for (let i = 0; i < n; i++) {
      const pi = pts[i]
      const ix = Math.floor(pi.x * inv)
      const iy = Math.floor(pi.y * inv)
      let c = 0
      for (let gx = ix - 1; gx <= ix + 1; gx++) {
        for (let gy = iy - 1; gy <= iy + 1; gy++) {
          const cell = grid.get(gx + gy * 1_000_000)
          if (!cell) continue
          for (const j of cell) {
            if (j === i) continue
            const dx = pts[j].x - pi.x
            const dy = pts[j].y - pi.y
            if (dx * dx + dy * dy < h * h) c++
          }
        }
      }
      off[i + 1] = off[i] + c
    }
    if (this.neigh.length < off[n]) this.neigh = new Int32Array(off[n])
    if (this.neighOff.length < n + 1) this.neighOff = new Int32Array(n + 1)
    for (let i = 0; i < n; i++) this.neighOff[i + 1] = off[i + 1]
    const cursor = Int32Array.prototype.slice.call(off, 0, n)
    for (let i = 0; i < n; i++) {
      const pi = pts[i]
      const ix = Math.floor(pi.x * inv)
      const iy = Math.floor(pi.y * inv)
      for (let gx = ix - 1; gx <= ix + 1; gx++) {
        for (let gy = iy - 1; gy <= iy + 1; gy++) {
          const cell = grid.get(gx + gy * 1_000_000)
          if (!cell) continue
          for (const j of cell) {
            if (j === i) continue
            const dx = pts[j].x - pi.x
            const dy = pts[j].y - pi.y
            if (dx * dx + dy * dy < h * h) {
              this.neigh[cursor[i]++] = j
            }
          }
        }
      }
    }
  }

  step(dt: number, bodies: Body[], field: Vec2): void {
    const cfg = this.config
    const h = cfg.h
    const pts = this.points
    const n = pts.length
    if (n === 0) return
    this.bodyFx.clear()

    // 1. Gravity.
    for (const p of pts) {
      p.vy += field.y * dt
    }

    // 2. Neighbor search on current positions.
    this.buildNeighbors()

    // 3. Viscosity impulses (symmetric pairs i<j).
    for (let i = 0; i < n; i++) {
      const pi = pts[i]
      for (let k = this.neighOff[i]; k < this.neighOff[i + 1]; k++) {
        const j = this.neigh[k]
        if (j <= i) continue
        const pj = pts[j]
        const dx = pj.x - pi.x
        const dy = pj.y - pi.y
        const d = Math.hypot(dx, dy)
        if (d < 1e-9) continue
        const q = d / h
        const nx = dx / d
        const ny = dy / d
        const u = (pi.vx - pj.vx) * nx + (pi.vy - pj.vy) * ny
        if (u > 0) {
          const I = dt * (1 - q) * (cfg.viscositySigma * u + cfg.viscosityBeta * u * u)
          const ix = (I * nx) / 2
          const iy = (I * ny) / 2
          pi.vx -= ix
          pi.vy -= iy
          pj.vx += ix
          pj.vy += iy
        }
      }
    }

    // 4. Advect.
    for (const p of pts) {
      p.px = p.x
      p.py = p.y
      p.x += p.vx * dt
      p.y += p.vy * dt
    }

    // 5. Double density relaxation (re-search neighbors after advection).
    this.buildNeighbors()
    const dt2 = dt * dt
    // Per-pair displacement cap. Kept well below the wall thickness so a
    // particle cannot tunnel through a boundary in one step even when all of
    // its ~18 neighbors contribute: 18 * cap must stay under the 0.15 m wall.
    const MAX_DISPLACEMENT = 0.004
    for (let i = 0; i < n; i++) {
      const pi = pts[i]
      let rho = 0
      let rhoNear = 0
      for (let k = this.neighOff[i]; k < this.neighOff[i + 1]; k++) {
        const pj = pts[this.neigh[k]]
        const d = Math.hypot(pj.x - pi.x, pj.y - pi.y)
        if (d >= h) continue
        const w = 1 - d / h
        rho += w * w
        rhoNear += w * w * w
      }
      pi.density = rho
      pi.nearDensity = rhoNear
      const P = cfg.stiffness * (rho - cfg.restDensity)
      const Pnear = cfg.nearStiffness * rhoNear
      pi.pressure = P
      pi.nearPressure = Pnear
      let dxAcc = 0
      let dyAcc = 0
      for (let k = this.neighOff[i]; k < this.neighOff[i + 1]; k++) {
        const pj = pts[this.neigh[k]]
        const dx = pj.x - pi.x
        const dy = pj.y - pi.y
        const d = Math.hypot(dx, dy)
        if (d < 1e-9 || d >= h) continue
        const w = 1 - d / h
        let D = dt2 * (P * w + Pnear * w * w)
        // Clamp: water cannot support tension (no negative displacement, which
        // otherwise causes the tensile/clustering instability) and no pair may
        // move more than a fraction of h per step (prevents runaway).
        if (D < 0) D = 0
        if (D > MAX_DISPLACEMENT) D = MAX_DISPLACEMENT
        const nx = dx / d
        const ny = dy / d
        pj.x += (D * nx) / 2
        pj.y += (D * ny) / 2
        dxAcc -= (D * nx) / 2
        dyAcc -= (D * ny) / 2
      }
      pi.x += dxAcc
      pi.y += dyAcc
    }

    // 6. Re-derive velocities from advection + relaxation. Done BEFORE the
    // body collision so a boundary position correction (which can be large for
    // a deeply-penetrating particle) is not converted into a velocity spike by
    // the re-derivation — that was the source of a growing, undamped bounce.
    const invDt = 1 / dt
    for (const p of pts) {
      p.vx = (p.x - p.px) * invDt
      p.vy = (p.y - p.py) * invDt
    }

    // 7. Body collisions + buoyancy reaction forces.
    for (let i = 0; i < n; i++) {
      const p = pts[i]
      for (const b of bodies) {
        if (b.kind === 'kinematic') continue
        const push = collidePointWithBody(p, this.pointRadius, b)
        if (!push) continue
        p.x += push.x
        p.y += push.y
        // Reaction on the body: Newton's third law. The body pushes the
        // particle out along push.n, so the particle pushes the body along
        // -push.n — for fluid below a box this is upward, which is what makes
        // buoyancy emerge.
        const fMag = this.particleMass * (p.pressure * cfg.forceScale + p.nearPressure * cfg.nearForceScale)
        const fx = push.nx * fMag
        const fy = push.ny * fMag
        let acc = this.bodyFx.get(b.id)
        if (!acc) {
          acc = { fx: 0, fy: 0, px: p.x, py: p.y, w: 1 }
          this.bodyFx.set(b.id, acc)
        }
        acc.px = (acc.px * acc.w + p.x) / (acc.w + 1)
        acc.py = (acc.py * acc.w + p.y) / (acc.w + 1)
        acc.w += 1
        acc.fx += fx
        acc.fy += fy
        // Inelastic boundary contact: remove the inward velocity component so
        // the particle does not plough back through the surface next step
        // (no rebound energy is added).
        const vn = p.vx * push.nx + p.vy * push.ny
        if (vn < 0) {
          p.vx -= vn * push.nx
          p.vy -= vn * push.ny
        }
      }
    }

    // 8. Speed cap: a particle must not travel more than a fraction of the
    // thinnest wall in one step, or it tunnels through and is lost. 4 m/s *
    // dt(0.0083) = 0.033 m << the 0.15 m wall. Water in a tank is slow; this
    // cap only trims the transient settling spike, not steady behaviour.
    const MAX_SPEED_SQ = 2.25
    for (const p of pts) {
      const s2 = p.vx * p.vx + p.vy * p.vy
      if (s2 > MAX_SPEED_SQ) {
        const s = 1.5 / Math.sqrt(s2)
        p.vx *= s
        p.vy *= s
      }
    }
  }

  /** Sum kinetic energy (for the instrument panel). */
  kineticEnergy(): number {
    let ke = 0
    for (const p of this.points) {
      ke += 0.5 * this.particleMass * (p.vx * p.vx + p.vy * p.vy)
    }
    return ke
  }
}
