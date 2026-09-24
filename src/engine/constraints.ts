import { Vec2 } from './vec'
import { Body } from './body'

/**
 * Constraint (joint) suite. `a`/`b` may be null, meaning a fixed anchor in
 * world space (the local vector then stores a world position).
 *
 * Impulse joints (distance, rope, revolute, prismatic, weld, mouse) enforce
 * velocity-level constraints inside the solver loop. Springs are force-based
 * (Hooke's law) and are applied in the force phase via `applyForces`.
 */
export abstract class Constraint {
  a: Body | null
  b: Body | null
  /** anchor in a's local frame, or world position when a === null */
  localA: Vec2
  /** anchor in b's local frame, or world position when b === null */
  localB: Vec2
  label = ''

  constructor(a: Body | null, b: Body | null, anchorA: Vec2, anchorB: Vec2) {
    this.a = a
    this.b = b
    this.localA = anchorA.clone()
    this.localB = anchorB.clone()
  }

  /** velocity-level solve, called once per solver iteration */
  abstract solveVelocity(dt: number): void

  /** force phase (default: none). Springs implement this. */
  applyForces(): void {}

  /**
   * Position-level correction, called once per step after the velocity
   * solve. Velocity-only constraints cannot produce centripetal forces
   * (a rod would let the body drift out along its tangent), so anchors are
   * projected back onto the constraint. No velocities are touched, so no
   * energy is injected.
   */
  correctPosition(_percent: number): void {}

  anchorA(): Vec2 {
    return this.a ? this.a.localToWorld(this.localA) : this.localA
  }

  anchorB(): Vec2 {
    return this.b ? this.b.localToWorld(this.localB) : this.localB
  }

  wake(): void {
    this.a?.wake()
    this.b?.wake()
  }

  /** Local anchor for a world-space point. */
  static localOf(body: Body, world: Vec2): Vec2 {
    const cos = Math.cos(-body.angle)
    const sin = Math.sin(-body.angle)
    const dx = world.x - body.pos.x
    const dy = world.y - body.pos.y
    return new Vec2(dx * cos - dy * sin, dx * sin + dy * cos)
  }
}

function pointVelocity(body: Body, r: { x: number; y: number }): { x: number; y: number } {
  // velocity of the point at offset r from center of mass
  return {
    x: body.vel.x - body.angVel * r.y,
    y: body.vel.y + body.angVel * r.x,
  }
}

// ------------------------------------------------------------------ distance

/** Rigid rod: holds the anchors exactly at `rest` apart. */
export class DistanceConstraint extends Constraint {
  rest: number

  constructor(a: Body | null, b: Body | null, anchorA: Vec2, anchorB: Vec2, rest?: number) {
    super(a, b, anchorA, anchorB)
    this.rest = rest ?? anchorA.dist(anchorB)
  }

  solveVelocity(_dt: number): void {
    const pa = this.anchorA()
    const pb = this.anchorB()
    let dx = pb.x - pa.x
    let dy = pb.y - pa.y
    const len = Math.hypot(dx, dy)
    if (len < 1e-9) return
    dx /= len
    dy /= len
    const va = this.a ? pointVelocity(this.a, subVec(pa, this.a.pos)) : ZERO
    const vb = this.b ? pointVelocity(this.b, subVec(pb, this.b.pos)) : ZERO
    const vn = (vb.x - va.x) * dx + (vb.y - va.y) * dy
    const ra = this.a ? subVec(pa, this.a.pos) : ZERO
    const rb = this.b ? subVec(pb, this.b.pos) : ZERO
    const kN = effMass(this.a, this.b, ra, rb, dx, dy)
    if (kN < 1e-9) return
    const j = -vn / kN
    applyPointImpulse(this.a, ra, dx, dy, -j)
    applyPointImpulse(this.b, rb, dx, dy, j)
  }

  correctPosition(percent: number): void {
    const pa = this.anchorA()
    const pb = this.anchorB()
    const dx = pb.x - pa.x
    const dy = pb.y - pa.y
    const len = Math.hypot(dx, dy)
    if (len < 1e-9) return
    const a = this.a
    const b = this.b
    const iA = a ? a.effInvMass : 0
    const iB = b ? b.effInvMass : 0
    const sum = iA + iB
    if (sum < 1e-9) return
    const diff = ((len - this.rest) / len) * percent
    if (a) {
      a.pos.x += dx * diff * (iA / sum)
      a.pos.y += dy * diff * (iA / sum)
    }
    if (b) {
      b.pos.x -= dx * diff * (iB / sum)
      b.pos.y -= dy * diff * (iB / sum)
    }
  }
}

// ---------------------------------------------------------------------- rope

/** Max-distance constraint: behaves like a rope, only pulls, never pushes. */
export class RopeConstraint extends DistanceConstraint {
  solveVelocity(dt: number): void {
    const pa = this.anchorA()
    const pb = this.anchorB()
    const dx = pb.x - pa.x
    const dy = pb.y - pa.y
    const len = Math.hypot(dx, dy)
    if (len <= this.rest) return // only active when taut
    super.solveVelocity(dt)
  }

  correctPosition(percent: number): void {
    const pa = this.anchorA()
    const pb = this.anchorB()
    if (Math.hypot(pb.x - pa.x, pb.y - pa.y) <= this.rest) return
    super.correctPosition(percent)
  }
}

// ------------------------------------------------------------------ revolute

/**
 * Pin joint with optional motor (target relative angular velocity) and
 * optional angle limits on the relative angle.
 */
export class RevoluteConstraint extends Constraint {
  motorOn = false
  motorSpeed = 0 // rad/s target for b relative to a
  motorForce = 100 // max torque
  angleLimit: [number, number] | null = null // relative angle [b - a] in radians
  limitForce = 100

  solveVelocity(dt: number): void {
    const pa = this.anchorA()
    const pb = this.anchorB()
    const a = this.a
    const b = this.b
    const ra = a ? subVec(pa, a.pos) : ZERO
    const rb = b ? subVec(pb, b.pos) : ZERO

    // Zero relative linear velocity at the anchor (two orthogonal impulses).
    const va = a ? pointVelocity(a, ra) : ZERO
    const vb = b ? pointVelocity(b, rb) : ZERO
    let rvx = vb.x - va.x
    let rvy = vb.y - va.y
    for (const [nx, ny] of [
      [1, 0],
      [0, 1],
    ] as const) {
      const kN = effMass(a, b, ra, rb, nx, ny)
      if (kN < 1e-9) continue
      const vn = rvx * nx + rvy * ny
      const j = -vn / kN
      applyPointImpulse(a, ra, nx, ny, -j)
      applyPointImpulse(b, rb, nx, ny, j)
      rvx = vb.x - va.x
      rvy = vb.y - va.y
    }

    // Angular: motor or limit.
    if (!a && !b) return
    const kAng = (a ? a.effInvInertia : 0) + (b ? b.effInvInertia : 0)
    if (kAng < 1e-9) return
    const relVel = (b ? b.angVel : 0) - (a ? a.angVel : 0)

    let target = NaN
    let maxJ = Infinity
    if (this.angleLimit) {
      const [lo, hi] = this.angleLimit
      const rel = wrapAngle((b ? b.angle : 0) - (a ? a.angle : 0))
      if (rel <= lo && relVel < 0) target = 0
      else if (rel >= hi && relVel > 0) target = 0
      if (!Number.isNaN(target)) maxJ = (this.limitForce * dt) / Math.max(kAng, 1e-9)
    } else if (this.motorOn) {
      target = this.motorSpeed
      maxJ = (this.motorForce * dt) / Math.max(kAng, 1e-9)
    }
    if (Number.isNaN(target)) return
    let j = (target - relVel) / kAng
    j = Math.max(-maxJ, Math.min(maxJ, j))
    if (a) a.angVel -= j * a.effInvInertia
    if (b) b.angVel += j * b.effInvInertia
  }

  correctPosition(percent: number): void {
    projectAnchorsTogether(this.a, this.b, this.localA, this.localB, percent)
  }
}

// ----------------------------------------------------------------- prismatic

/** Slider: b translates along a fixed world axis relative to a. */
export class PrismaticConstraint extends Constraint {
  axis: Vec2 // world-space unit axis
  maxForce = 200

  constructor(a: Body | null, b: Body | null, anchorA: Vec2, anchorB: Vec2, axis: Vec2) {
    super(a, b, anchorA, anchorB)
    this.axis = axis.clone().normalize()
  }

  correctPosition(percent: number): void {
    const pa = this.anchorA()
    const pb = this.anchorB()
    const dx = pb.x - pa.x
    const dy = pb.y - pa.y
    const along = dx * this.axis.x + dy * this.axis.y
    const px = dx - along * this.axis.x
    const py = dy - along * this.axis.y
    if (px * px + py * py < 1e-12) return
    const a = this.a
    const b = this.b
    const iA = a ? a.effInvMass : 0
    const iB = b ? b.effInvMass : 0
    const sum = iA + iB
    if (sum < 1e-9) return
    if (a) {
      a.pos.x += px * (iA / sum) * percent
      a.pos.y += py * (iA / sum) * percent
    }
    if (b) {
      b.pos.x -= px * (iB / sum) * percent
      b.pos.y -= py * (iB / sum) * percent
    }
  }

  solveVelocity(dt: number): void {
    const a = this.a
    const b = this.b
    const pa = this.anchorA()
    const pb = this.anchorB()
    const ra = a ? subVec(pa, a.pos) : ZERO
    const rb = b ? subVec(pb, b.pos) : ZERO
    const nx = -this.axis.y
    const ny = this.axis.x
    const va = a ? pointVelocity(a, ra) : ZERO
    const vb = b ? pointVelocity(b, rb) : ZERO
    const vn = (vb.x - va.x) * nx + (vb.y - va.y) * ny
    const kN = effMass(a, b, ra, rb, nx, ny)
    if (kN >= 1e-9) {
      const j = -vn / kN
      applyPointImpulse(a, ra, nx, ny, -j)
      applyPointImpulse(b, rb, nx, ny, j)
    }
    // Lock relative rotation.
    const kAng = (a ? a.effInvInertia : 0) + (b ? b.effInvInertia : 0)
    if (kAng >= 1e-9) {
      const relVel = (b ? b.angVel : 0) - (a ? a.angVel : 0)
      let j = -relVel / kAng
      const maxJ = (this.maxForce * dt) / Math.max(kAng, 1e-9)
      j = Math.max(-maxJ, Math.min(maxJ, j))
      if (a) a.angVel -= j * a.effInvInertia
      if (b) b.angVel += j * b.effInvInertia
    }
  }
}

// --------------------------------------------------------------------- weld

/** Freezes the relative pose at the anchor (linear + angular). */
export class WeldConstraint extends Constraint {
  maxForce = 200

  correctPosition(percent: number): void {
    projectAnchorsTogether(this.a, this.b, this.localA, this.localB, percent)
  }

  solveVelocity(dt: number): void {
    const a = this.a
    const b = this.b
    const pa = this.anchorA()
    const pb = this.anchorB()
    const ra = a ? subVec(pa, a.pos) : ZERO
    const rb = b ? subVec(pb, b.pos) : ZERO
    const va = a ? pointVelocity(a, ra) : ZERO
    const vb = b ? pointVelocity(b, rb) : ZERO
    let rvx = vb.x - va.x
    let rvy = vb.y - va.y
    for (const [nx, ny] of [
      [1, 0],
      [0, 1],
    ] as const) {
      const kN = effMass(a, b, ra, rb, nx, ny)
      if (kN < 1e-9) continue
      const vn = rvx * nx + rvy * ny
      const j = -vn / kN
      applyPointImpulse(a, ra, nx, ny, -j)
      applyPointImpulse(b, rb, nx, ny, j)
      rvx = vb.x - va.x
      rvy = vb.y - va.y
    }
    const kAng = (a ? a.effInvInertia : 0) + (b ? b.effInvInertia : 0)
    if (kAng < 1e-9) return
    const relVel = (b ? b.angVel : 0) - (a ? a.angVel : 0)
    let j = -relVel / kAng
    const maxJ = (this.maxForce * dt) / Math.max(kAng, 1e-9)
    j = Math.max(-maxJ, Math.min(maxJ, j))
    if (a) a.angVel -= j * a.effInvInertia
    if (b) b.angVel += j * b.effInvInertia
  }
}

// -------------------------------------------------------------------- spring

/**
 * Hooke's-law spring-damper: F = -k(x - rest) - c*v_rel along the axis.
 * Force-based, so stiffness is limited by stability: k < 4*m/dt^2.
 */
export class SpringConstraint extends Constraint {
  stiffness: number // k (N/m)
  damping: number // c (N*s/m)
  rest: number
  /** spring potential energy for the instrument panel */
  stretch = 0

  constructor(a: Body | null, b: Body | null, anchorA: Vec2, anchorB: Vec2, k = 200, c = 1, rest?: number) {
    super(a, b, anchorA, anchorB)
    this.stiffness = k
    this.damping = c
    this.rest = rest ?? anchorA.dist(anchorB)
  }

  applyForces(): void {
    const pa = this.anchorA()
    const pb = this.anchorB()
    let dx = pb.x - pa.x
    let dy = pb.y - pa.y
    const len = Math.hypot(dx, dy)
    if (len < 1e-9) return
    dx /= len
    dy /= len
    const ra = this.a ? subVec(pa, this.a.pos) : ZERO
    const rb = this.b ? subVec(pb, this.b.pos) : ZERO
    const va = this.a ? pointVelocity(this.a, ra) : ZERO
    const vb = this.b ? pointVelocity(this.b, rb) : ZERO
    const rvn = (vb.x - va.x) * dx + (vb.y - va.y) * dy
    const f = (-this.stiffness * (len - this.rest) - this.damping * rvn) * dx
    const fy = (-this.stiffness * (len - this.rest) - this.damping * rvn) * dy
    this.stretch = len - this.rest
    if (this.a) {
      this.a.force.x -= f
      this.a.force.y -= fy
      this.a.torque -= ra.x * fy - ra.y * f
    }
    if (this.b) {
      this.b.force.x += f
      this.b.force.y += fy
      this.b.torque += rb.x * fy - rb.y * f
    }
  }

  potentialEnergy(): number {
    return 0.5 * this.stiffness * this.stretch * this.stretch
  }

  solveVelocity(_dt: number): void {
    // force-based: nothing to do at velocity level
  }
}

// --------------------------------------------------------------------- mouse

/**
 * Live grab: drives a body point toward a moving target with a stiff
 * impulse clamp, so dragging feels direct and flinging transfers real
 * velocity (the tool sets `targetVel` from pointer motion).
 */
export class MouseConstraint extends Constraint {
  target = new Vec2()
  targetVel = new Vec2()
  maxForce = 5000
  /** current accumulated normal impulse, for impact sounds */
  lastImpulse = 0

  constructor(body: Body, anchor: Vec2, target: Vec2) {
    super(body, null, anchor, target)
    this.target = target.clone()
  }

  solveVelocity(dt: number): void {
    const body = this.a
    if (!body) return
    const pa = this.anchorA()
    const r = subVec(pa, body.pos)
    const vp = pointVelocity(body, r)
    // Desired point velocity: pointer velocity plus a pull-back term so the
    // body tracks the target without overshooting.
    const pull = 20
    const dvx = this.targetVel.x + (this.target.x - pa.x) * pull
    const dvy = this.targetVel.y + (this.target.y - pa.y) * pull
    // Impulse along the error direction, clamped by maxForce.
    const ex = dvx - vp.x
    const ey = dvy - vp.y
    const elen = Math.hypot(ex, ey)
    if (elen < 1e-9) return
    const nx = ex / elen
    const ny = ey / elen
    const kN = body.effInvMass + body.effInvInertia * (r.x * ny - r.y * nx) ** 2
    if (kN < 1e-9) return
    const j = Math.min(elen / kN, this.maxForce * dt)
    applyPointImpulse(body, r, nx, ny, j)
    this.lastImpulse = j
  }
}

// ----------------------------------------------------------------- utilities

const ZERO = { x: 0, y: 0 }

function subVec(p: Vec2, o: Vec2): { x: number; y: number } {
  return { x: p.x - o.x, y: p.y - o.y }
}

/** Move both bodies so their anchors coincide, weighted by inv mass. */
function projectAnchorsTogether(
  a: Body | null,
  b: Body | null,
  la: Vec2,
  lb: Vec2,
  percent: number,
): void {
  const pa = a ? a.localToWorld(la) : la
  const pb = b ? b.localToWorld(lb) : lb
  const dx = pb.x - pa.x
  const dy = pb.y - pa.y
  if (dx * dx + dy * dy < 1e-12) return
  const iA = a ? a.effInvMass : 0
  const iB = b ? b.effInvMass : 0
  const sum = iA + iB
  if (sum < 1e-9) return
  if (a) {
    a.pos.x += dx * (iA / sum) * percent
    a.pos.y += dy * (iA / sum) * percent
  }
  if (b) {
    b.pos.x -= dx * (iB / sum) * percent
    b.pos.y -= dy * (iB / sum) * percent
  }
}

/** Effective mass for an impulse along unit axis (nx, ny) at offsets ra/rb. */
function effMass(
  a: Body | null,
  b: Body | null,
  ra: { x: number; y: number },
  rb: { x: number; y: number },
  nx: number,
  ny: number,
): number {
  const raC = ra.x * ny - ra.y * nx
  const rbC = rb.x * ny - rb.y * nx
  return (
    (a ? a.effInvMass : 0) +
    (b ? b.effInvMass : 0) +
    (a ? a.effInvInertia * raC * raC : 0) +
    (b ? b.effInvInertia * rbC * rbC : 0)
  )
}

function applyPointImpulse(
  body: Body | null,
  r: { x: number; y: number },
  nx: number,
  ny: number,
  j: number,
): void {
  if (!body || j === 0) return
  body.vel.x += nx * j * body.effInvMass
  body.vel.y += ny * j * body.effInvMass
  body.angVel += body.effInvInertia * (r.x * (ny * j) - r.y * (nx * j))
}

export function wrapAngle(a: number): number {
  while (a > Math.PI) a -= 2 * Math.PI
  while (a < -Math.PI) a += 2 * Math.PI
  return a
}
