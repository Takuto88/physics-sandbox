import { Vec2 } from './vec'
import { Shape, massProperties, shapeRadius } from './shapes'
import { Material, DEFAULT_MATERIAL } from './material'
export type { Material }

let nextId = 1

export type BodyKind = 'dynamic' | 'static' | 'kinematic'

/**
 * Rigid body state. `pos` is the center of mass; `prevPos`/`prevAngle` hold
 * the state from the previous physics tick for render interpolation.
 *
 * Sleeping bodies report effectiveInvMass = 0 so the solver treats them as
 * static; any impulse above a threshold wakes them.
 */
export class Body {
  id: number
  kind: BodyKind = 'dynamic'
  shape: Shape
  material: Material
  label = ''

  pos = new Vec2()
  angle = 0
  prevPos = new Vec2()
  prevAngle = 0

  vel = new Vec2()
  angVel = 0

  /** Accumulated force/torque for the current step. Zeroed at step start. */
  force = new Vec2()
  torque = 0

  mass = 1
  invMass = 1
  inertia = 1
  invInertia = 1

  linearDamping = 0
  angularDamping = 0
  fixedRotation = false

  sleeping = false
  sleepTimer = 0

  /** World-space axis-aligned bounding box of the shape (kept by World). */
  aabb = { minX: 0, minY: 0, maxX: 0, maxY: 0 }
  aabbDirty = true

  /** Small + fast bodies get swept collision so they cannot tunnel. */
  ccd: boolean
  /** Rendering metadata (trail on/off, etc.) lives here, not in the engine. */
  trailEnabled = true
  userData: Record<string, unknown> = {}

  constructor(shape: Shape, material?: Partial<Material>) {
    this.id = nextId++
    this.shape = shape
    this.material = { ...DEFAULT_MATERIAL, ...material }
    this.recomputeMass()
    this.ccd = this.shape.type === 'circle' && this.shape.radius < 0.1
  }

  recomputeMass(): void {
    const { mass, inertia } = massProperties(this.shape, this.material.density)
    if (this.kind === 'static' || this.kind === 'kinematic') {
      this.mass = 0
      this.invMass = 0
      this.inertia = 0
      this.invInertia = 0
    } else {
      this.mass = Math.max(mass, 1e-6)
      this.invMass = 1 / this.mass
      this.inertia = Math.max(inertia, 1e-9)
      this.invInertia = 1 / this.inertia
      if (this.fixedRotation) this.invInertia = 0
    }
  }

  setKind(k: BodyKind): void {
    this.kind = k
    this.recomputeMass()
    this.wake()
  }

  isDynamic(): boolean {
    return this.kind === 'dynamic'
  }

  /**
   * Inv mass to use in the solver: 0 while sleeping, and 0 for static and
   * kinematic bodies (they are driven by their velocity, not by impulses).
   */
  get effInvMass(): number {
    if (this.kind !== 'dynamic' || this.sleeping) return 0
    return this.invMass
  }

  get effInvInertia(): number {
    if (this.kind !== 'dynamic' || this.sleeping) return 0
    return this.invInertia
  }

  applyForce(f: Vec2, at?: Vec2): void {
    this.force.add(f)
    if (at) {
      const rx = at.x - this.pos.x
      const ry = at.y - this.pos.y
      this.torque += rx * f.y - ry * f.x
    }
    this.wake()
  }

  /** Impulse J (N*s) applied at point `at` (default: center of mass). */
  applyImpulse(j: Vec2, at?: Vec2): void {
    this.vel.addScaled(j, this.invMass)
    if (at) {
      const rx = at.x - this.pos.x
      const ry = at.y - this.pos.y
      this.angVel += this.invInertia * (rx * j.y - ry * j.x)
    }
    this.wake()
  }

  wake(): void {
    if (this.sleeping) {
      this.sleeping = false
      this.sleepTimer = 0
    }
  }

  /** Update the world AABB; call after pos/angle changes. */
  updateAABB(): void {
    const s = this.shape
    const a = this.angle
    const cos = Math.cos(a)
    const sin = Math.sin(a)
    const p = this.pos
    let minX = Infinity
    let minY = Infinity
    let maxX = -Infinity
    let maxY = -Infinity
    const consider = (x: number, y: number) => {
      const wx = p.x + x * cos - y * sin
      const wy = p.y + x * sin + y * cos
      if (wx < minX) minX = wx
      if (wy < minY) minY = wy
      if (wx > maxX) maxX = wx
      if (wy > maxY) maxY = wy
    }
    if (s.type === 'circle') {
      const r = s.radius
      consider(r, 0)
      consider(-r, 0)
      consider(0, r)
      consider(0, -r)
    } else {
      for (const v of s.verts) consider(v.x, v.y)
    }
    const b = this.aabb
    b.minX = minX
    b.minY = minY
    b.maxX = maxX
    b.maxY = maxY
  }

  staticWorldPoint(local: Vec2): Vec2 {
    const cos = Math.cos(this.angle)
    const sin = Math.sin(this.angle)
    return new Vec2(
      this.pos.x + local.x * cos - local.y * sin,
      this.pos.y + local.x * sin + local.y * cos,
    )
  }

  localToWorld(v: Vec2): Vec2 {
    const cos = Math.cos(this.angle)
    const sin = Math.sin(this.angle)
    return new Vec2(
      this.pos.x + v.x * cos - v.y * sin,
      this.pos.y + v.x * sin + v.y * cos,
    )
  }

  /** Velocity of a local point (for solver and grab tools). */
  pointVelocity(local: Vec2, out?: Vec2): Vec2 {
    const vx = this.vel.x - this.angVel * local.y
    const vy = this.vel.y + this.angVel * local.x
    return (out ?? new Vec2()).set(vx, vy)
  }

  worldRadius(): number {
    return shapeRadius(this.shape)
  }
}

/** Reset the id counter (used by scene loading to keep deterministic ids). */
export function resetBodyIds(): void {
  nextId = 1
}
