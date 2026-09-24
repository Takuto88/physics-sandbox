import { Vec2 } from './vec'
import { Body } from './body'

export type GravityMode = 'uniform' | 'nbody' | 'none'

/**
 * Gravity models.
 *
 * uniform: constant field (default: Earth, 9.81 m/s^2 downward).
 * nbody:   true Newtonian inverse-square, every massive body attracts every
 *          other. Used by the orbital module; G is in scene units (the
 *          orbital preset works in AU / years / solar masses where
 *          G = 4*pi^2 so that M = 1, a = 1 AU gives P = 1 year).
 */
export class Gravity {
  mode: GravityMode = 'uniform'
  field = new Vec2(0, -9.81)
  G = 4 * Math.PI * Math.PI
  /** softening length to avoid singular forces at r -> 0 */
  softening = 0.01

  /** Apply gravity forces to all bodies. Returns nothing; accumulates on body.force. */
  apply(bodies: Body[]): void {
    if (this.mode === 'none') return
    if (this.mode === 'uniform') {
      for (const b of bodies) {
        if (!b.isDynamic() || b.sleeping) continue
        b.force.x += b.mass * this.field.x
        b.force.y += b.mass * this.field.y
      }
      return
    }
    // Newtonian N-body: O(n^2) over massive bodies. Orbital scenes are small.
    // Gravitational mass (bodyMass) and inertial mass (body.mass) are
    // decoupled: the acceleration a = G*m_grav_other/r^2 is applied as
    // F = a * m_inertial so that F*invMass recovers the true acceleration.
    const n = bodies.length
    for (let i = 0; i < n; i++) {
      const bi = bodies[i]
      // static bodies still gravitate (e.g. a fixed sun); they simply
      // never receive force (guarded below).
      for (let j = i + 1; j < n; j++) {
        const bj = bodies[j]
        const mi = bodyMass(bi)
        const mj = bodyMass(bj)
        if (mi * mj <= 0) continue
        const dx = bj.pos.x - bi.pos.x
        const dy = bj.pos.y - bi.pos.y
        const r2 = dx * dx + dy * dy + this.softening * this.softening
        const r = Math.sqrt(r2)
        if (bi.isDynamic() && !bi.sleeping) {
          const acc = (this.G * mj) / r2
          bi.force.x += (acc * dx * bi.mass) / r
          bi.force.y += (acc * dy * bi.mass) / r
        }
        if (bj.isDynamic() && !bj.sleeping) {
          const acc = (this.G * mi) / r2
          bj.force.x -= (acc * dx * bj.mass) / r
          bj.force.y -= (acc * dy * bj.mass) / r
        }
      }
    }
  }

  /** Gravitational potential energy of the scene (for the instrument panel). */
  potentialEnergy(bodies: Body[]): number {
    if (this.mode === 'uniform') {
      let pe = 0
      for (const b of bodies) {
        if (!b.isDynamic()) continue
        // PE = -m * g . p; with g = (0, -9.81) this is m * 9.81 * y.
        pe -= b.mass * (this.field.x * b.pos.x + this.field.y * b.pos.y)
      }
      return pe
    }
    if (this.mode === 'nbody') {
      let pe = 0
      const n = bodies.length
      for (let i = 0; i < n; i++) {
        for (let j = i + 1; j < n; j++) {
          const mi = bodyMass(bodies[i])
          const mj = bodyMass(bodies[j])
          if (mi * mj <= 0) continue
          const dx = bodies[j].pos.x - bodies[i].pos.x
          const dy = bodies[j].pos.y - bodies[i].pos.y
          const r = Math.sqrt(dx * dx + dy * dy + this.softening * this.softening)
          pe -= (this.G * mi * mj) / r
        }
      }
      return pe
    }
    return 0
  }
}

/**
 * Gravitational mass of a body. `userData.gravMass` overrides the inertial
 * mass for any body (static or dynamic); without it the computed inertial
 * mass is used, so a "static sun" still attracts.
 */
function bodyMass(b: Body): number {
  const g = b.userData['gravMass'] as number | undefined
  return g !== undefined ? g : b.mass
}
