import { Body } from './body'
import { Manifold, ContactPoint } from './contact'
import { Constraint } from './constraints'

export interface SolverParams {
  /** velocity iterations per step */
  iterations: number
  /** |relative normal velocity| below which restitution is ignored (m/s) */
  restitutionThreshold: number
  /** allowed penetration before Baumgarte correction kicks in (m) */
  slop: number
  /** Baumgarte coefficient (0..1), position correction rate */
  baumgarte: number
}

export const DEFAULT_SOLVER: SolverParams = {
  iterations: 10,
  restitutionThreshold: 0.5,
  slop: 0.005,
  baumgarte: 0.2,
}

/**
 * Sequential-impulse solver with warm starting and Baumgarte position bias.
 * Restitution is applied only when the approach speed exceeds a threshold,
 * which is what kills micro-bounce jitter on settled stacks.
 *
 * Warm-started impulses are clamped so they can never separate a contact:
 * the one-sided accumulated-impulse clamp in `solve` can only add, so an
 * unclamped warm-start of a previous step's bounce re-injects that energy
 * into a separating or re-colliding pair on every step.
 */
export class Solver {
  private warm = new Map<string, { jn: number; jt: number }>()

  /**
   * Prepare per-contact state: record incoming normal velocity (for
   * restitution) and warm-start accumulated impulses from the previous step.
   *
   * Warm starting is applied only to quasi-static contacts (both bodies slow
   * and not separating): it accelerates convergence of resting stacks, but on
   * a fast, colliding pair the one-sided accumulated-impulse clamp in `solve`
   * cannot remove the previous step's bounce, so re-applying it injects
   * energy on every step.
   */
  private static readonly WARM_SPEED = 0.5

  prepare(m: Manifold): void {
    const { a, b } = m
    for (const pt of m.points) {
      const raX = pt.pos.x - a.pos.x
      const raY = pt.pos.y - a.pos.y
      const rbX = pt.pos.x - b.pos.x
      const rbY = pt.pos.y - b.pos.y
      const rvx = b.vel.x - b.angVel * rbY - (a.vel.x - a.angVel * raY)
      const rvy = b.vel.y + b.angVel * rbX - (a.vel.y + a.angVel * raX)
      pt.vn0 = rvx * pt.normal.x + rvy * pt.normal.y
    }
    const aSlow = a.vel.len() < Solver.WARM_SPEED
    const bSlow = b.vel.len() < Solver.WARM_SPEED
    const quasiStatic = aSlow && bSlow
    for (const pt of m.points) {
      const key = this.key(a, b, pt)
      const w = quasiStatic ? this.warm.get(key) : undefined
      if (!w) {
        pt.normalImpulse = 0
        pt.tangentImpulse = 0
        continue
      }
      const raX = pt.pos.x - a.pos.x
      const raY = pt.pos.y - a.pos.y
      const rbX = pt.pos.x - b.pos.x
      const rbY = pt.pos.y - b.pos.y
      const rnA = raX * pt.normal.y - raY * pt.normal.x
      const rnB = rbX * pt.normal.y - rbY * pt.normal.x
      const kN =
        a.effInvMass + b.effInvMass + a.effInvInertia * rnA * rnA + b.effInvInertia * rnB * rnB
      // A support impulse may only cancel approach, never create separation:
      // after applying Jn the relative normal velocity is vn0 + Jn/kN, which
      // must stay <= 0. This strips the previous step's bounce out of the
      // warm-start while keeping resting-contact support (small Jn, small |vn0|).
      let jn = pt.vn0 >= 0 ? 0 : Math.min(w.jn, -pt.vn0 * kN)
      const maxF = pt.friction * jn
      const jt = Math.max(-maxF, Math.min(maxF, w.jt))
      pt.normalImpulse = jn
      pt.tangentImpulse = jt
      this.applyImpulse(a, b, pt.pos, pt.normal, jn, jt)
    }
  }

  private key(a: Body, b: Body, pt: ContactPoint): string {
    return `${a.id}:${b.id}:${pt.id}`
  }

  private applyImpulse(a: Body, b: Body, at: { x: number; y: number }, n: { x: number; y: number }, jn: number, jt: number): void {
    const px = n.x * jn - n.y * jt
    const py = n.y * jn + n.x * jt
    const aX = at.x - a.pos.x
    const aY = at.y - a.pos.y
    const bX = at.x - b.pos.x
    const bY = at.y - b.pos.y
    const ia = a.effInvMass
    const ib = b.effInvMass
    a.vel.x -= px * ia
    a.vel.y -= py * ia
    a.angVel -= a.effInvInertia * (aX * py - aY * px)
    b.vel.x += px * ib
    b.vel.y += py * ib
    b.angVel += b.effInvInertia * (bX * py - bY * px)
  }

  solve(m: Manifold, dt: number, params: SolverParams): void {
    const { a, b } = m
    for (const pt of m.points) {
      const raX = pt.pos.x - a.pos.x
      const raY = pt.pos.y - a.pos.y
      const rbX = pt.pos.x - b.pos.x
      const rbY = pt.pos.y - b.pos.y

      // --- normal impulse ---
      let rvx = b.vel.x - b.angVel * rbY - (a.vel.x - a.angVel * raY)
      let rvy = b.vel.y + b.angVel * rbX - (a.vel.y + a.angVel * raX)
      let vn = rvx * pt.normal.x + rvy * pt.normal.y

      let bounce = 0
      if (-pt.vn0 > params.restitutionThreshold) {
        bounce = pt.restitution * -pt.vn0
      }
      const bias = (params.baumgarte / dt) * Math.max(0, pt.penetration - params.slop) + bounce

      const rnA = raX * pt.normal.y - raY * pt.normal.x
      const rnB = rbX * pt.normal.y - rbY * pt.normal.x
      const kN =
        a.effInvMass + b.effInvMass + a.effInvInertia * rnA * rnA + b.effInvInertia * rnB * rnB
      if (kN < 1e-9) continue
      let j = (-vn + bias) / kN
      const newImp = Math.max(0, pt.normalImpulse + j)
      j = newImp - pt.normalImpulse
      pt.normalImpulse = newImp

      a.vel.x -= pt.normal.x * j * a.effInvMass
      a.vel.y -= pt.normal.y * j * a.effInvMass
      a.angVel -= a.effInvInertia * (raX * (pt.normal.y * j) - raY * (pt.normal.x * j))
      b.vel.x += pt.normal.x * j * b.effInvMass
      b.vel.y += pt.normal.y * j * b.effInvMass
      b.angVel += b.effInvInertia * (rbX * (pt.normal.y * j) - rbY * (pt.normal.x * j))

      // --- friction impulse ---
      rvx = b.vel.x - b.angVel * rbY - (a.vel.x - a.angVel * raY)
      rvy = b.vel.y + b.angVel * rbX - (a.vel.y + a.angVel * raX)
      const vt = rvx * pt.tangent.x + rvy * pt.tangent.y
      const rtA = raX * pt.tangent.y - raY * pt.tangent.x
      const rtB = rbX * pt.tangent.y - rbY * pt.tangent.x
      const kT = a.effInvMass + b.effInvMass + a.effInvInertia * rtA * rtA + b.effInvInertia * rtB * rtB
      if (kT < 1e-9) continue
      let jt = -vt / kT
      const maxF = pt.friction * pt.normalImpulse
      const newJt = Math.max(-maxF, Math.min(maxF, pt.tangentImpulse + jt))
      jt = newJt - pt.tangentImpulse
      pt.tangentImpulse = newJt

      const tx = pt.tangent.x * jt
      const ty = pt.tangent.y * jt
      a.vel.x -= tx * a.effInvMass
      a.vel.y -= ty * a.effInvMass
      a.angVel -= a.effInvInertia * (raX * ty - raY * tx)
      b.vel.x += tx * b.effInvMass
      b.vel.y += ty * b.effInvMass
      b.angVel += b.effInvInertia * (rbX * ty - rbY * tx)
    }
  }

  /** Persist accumulated impulses for warm starting next step. */
  commit(manifolds: Manifold[]): void {
    for (const m of manifolds) {
      for (const pt of m.points) {
        if (pt.normalImpulse > 1e-9) {
          this.warm.set(this.key(m.a, m.b, pt), { jn: pt.normalImpulse, jt: pt.tangentImpulse })
        }
      }
    }
    // Drop stale entries (beyond a sanity cap) so the map stays small.
    if (this.warm.size > 8192) this.warm.clear()
  }

  clear(): void {
    this.warm.clear()
  }
}

export type { Constraint }
