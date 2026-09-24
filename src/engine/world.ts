import { Vec2 } from './vec'
import { Body, resetBodyIds } from './body'
import { Shape, circleShape, boxShape } from './shapes'
import { Material } from './material'
import { Broadphase } from './broadphase'
import { generateManifold } from './narrowphase'
import { Solver } from './solver'
import { Constraint, SpringConstraint } from './constraints'
import { Gravity, GravityMode } from './gravity'
import { ParticleSystem } from './particles'
import { Cloth } from './softbody'
import { ShapeMatchingBlob } from './softbody'
import { Fluid } from './fluid'
import { RNG } from './rand'
import { sweepCircle } from './ccd'
import { Manifold } from './contact'

export const PHYSICS_HZ = 120
export const DT = 1 / PHYSICS_HZ

export type Integrator = 'euler' | 'verlet' | 'rk4'

export interface WorldConfig {
  gravityMode: GravityMode
  gravityX: number
  gravityY: number
  /** N-body constant (orbital preset sets G = 4*pi^2 in AU/yr/solar-mass units) */
  G: number
  softening: number
  /** linear drag coefficient per unit mass: a = -drag * v */
  airDrag: number
  /** user-facing time scale (0..4) */
  timeScale: number
  /** scene time scale (orbital acceleration); multiplied with timeScale */
  baseTimeScale: number
  /** integrator substeps per 120 Hz tick (orbital preset uses more) */
  substeps: number
  /**
   * 'verlet' (default): velocity Verlet, 2nd-order symplectic — total
   * energy stays bounded for lossless scenes, which the acceptance tests
   * require. 'euler': semi-implicit Euler, kept selectable because its
   * visible secular energy drift is itself a teaching tool.
   */
  integrator: Integrator
  solverIterations: number
  restitutionThreshold: number
  slop: number
  baumgarte: number
  seed: number
}

export const DEFAULT_CONFIG: WorldConfig = {
  gravityMode: 'uniform',
  gravityX: 0,
  gravityY: -9.81,
  G: 4 * Math.PI * Math.PI,
  softening: 0.001,
  airDrag: 0,
  timeScale: 1,
  baseTimeScale: 1,
  substeps: 1,
  integrator: 'verlet',
  solverIterations: 10,
  restitutionThreshold: 0.5,
  slop: 0.005,
  // kept small: the main position correction is the projection pass, and a
  // large velocity bias would inject energy on every bounce
  baumgarte: 0.1,
  seed: 1,
}

export interface EnergySample {
  ke: number
  pe: number
  total: number
  px: number
  py: number
  L: number
  t: number
}

export interface ContactEvent {
  x: number
  y: number
  impulse: number
  a: Body
  b: Body
}

export interface Emitter {
  x: number
  y: number
  shape: Shape
  material: Material
  /** bodies per second */
  rate: number
  /** total bodies to emit; -1 = infinite */
  total: number
  velBase: Vec2
  velSpread: number
  jitter: number
  elapsed: number
  acc: number
  emitted: number
  active: boolean
}

interface Snapshot {
  t: number
  data: Float32Array
  ids: number[]
}

/**
 * The simulation core. Owns all bodies, constraints, deformables, fluids and
 * particles, and advances them at a fixed 120 Hz timestep decoupled from the
 * render framerate (accumulator + interpolation alpha).
 */
export class World {
  config: WorldConfig
  rng: RNG
  gravity: Gravity
  bodies: Body[] = []
  constraints: Constraint[] = []
  cloths: Cloth[] = []
  blobs: ShapeMatchingBlob[] = []
  fluid: Fluid | null = null
  particles = new ParticleSystem(new RNG(0))
  emitters: Emitter[] = []

  private solver = new Solver()
  private broadphase = new Broadphase(2)
  private manifolds: Manifold[] = []

  time = 0
  accumulator = 0
  /** interpolation factor between previous and current state, 0..1 */
  alpha = 0
  running = false
  /** steps executed in the most recent tick */
  stepsThisTick = 0

  /** live energy/momentum instrumentation */
  energy: EnergySample = { ke: 0, pe: 0, total: 0, px: 0, py: 0, L: 0, t: 0 }
  /** ring buffer of energy samples (1 per physics tick) */
  history: EnergySample[] = []
  historyCapacity = PHYSICS_HZ * 30
  /** recorded body states for reverse scrubbing */
  private snapshots: Snapshot[] = []
  snapshotCapacity = PHYSICS_HZ * 6
  /** per-tick contact impulses, consumed by the UI (audio, sparks) */
  contactEvents: ContactEvent[] = []
  onContact: ((e: ContactEvent) => void) | null = null

  /** bodies currently selected (UI) */
  selected: Body[] = []

  constructor(config?: Partial<WorldConfig>) {
    this.config = { ...DEFAULT_CONFIG, ...config }
    this.rng = new RNG(this.config.seed)
    this.gravity = new Gravity()
    this.syncGravity()
  }

  syncGravity(): void {
    const c = this.config
    this.gravity.mode = c.gravityMode
    this.gravity.field.set(c.gravityX, c.gravityY)
    this.gravity.G = c.G
    this.gravity.softening = c.softening
  }

  // ------------------------------------------------------------- registry

  addBody(body: Body): Body {
    this.bodies.push(body)
    body.updateAABB()
    return body
  }

  removeBody(body: Body): void {
    const i = this.bodies.indexOf(body)
    if (i >= 0) this.bodies.splice(i, 1)
    for (const c of [...this.constraints]) {
      if (c.a === body || c.b === body) this.removeConstraint(c)
    }
    this.selected = this.selected.filter((b) => b !== body)
    this.solver.clear()
  }

  addConstraint(c: Constraint): void {
    this.constraints.push(c)
    this.solver.clear()
  }

  removeConstraint(c: Constraint): void {
    const i = this.constraints.indexOf(c)
    if (i >= 0) this.constraints.splice(i, 1)
    this.solver.clear()
  }

  addCloth(c: Cloth): void {
    this.cloths.push(c)
  }

  addBlob(b: ShapeMatchingBlob): void {
    this.blobs.push(b)
  }

  setFluid(f: Fluid | null): void {
    this.fluid = f
  }

  addEmitter(e: Emitter): void {
    this.emitters.push(e)
  }

  clearWorld(): void {
    this.bodies.length = 0
    this.constraints.length = 0
    this.cloths.length = 0
    this.blobs.length = 0
    this.fluid = null
    this.particles.clear()
    this.emitters.length = 0
    this.solver.clear()
    this.history.length = 0
    this.snapshots.length = 0
    this.time = 0
    this.accumulator = 0
  }

  // -------------------------------------------------------------- stepping

  /**
   * Advance the simulation by real elapsed seconds. Physics always runs at
   * 120 Hz; the accumulator decouples it from the render framerate.
   */
  tick(realDt: number): void {
    const scaled = realDt * this.config.timeScale * this.config.baseTimeScale
    this.accumulator += scaled
    let steps = 0
    const maxSteps = 8 // spiral-of-death guard
    while (this.accumulator >= DT && steps < maxSteps) {
      this.step(DT)
      this.accumulator -= DT
      steps++
    }
    if (steps === maxSteps) this.accumulator = 0
    this.stepsThisTick = steps
    this.alpha = this.accumulator / DT
  }

  /** Advance exactly one 120 Hz tick (single-step button). */
  singleStep(): void {
    this.step(DT)
    this.alpha = 0
  }

  /**
   * One fixed physics step of duration `dt`. Pipeline:
   *  forces (fluid reaction, gravity, drag, springs) -> velocity integrate
   *  -> broadphase -> narrowphase -> velocity solve -> CCD -> position
   *  integrate -> sleep bookkeeping -> deformables/particles -> instruments.
   */
  step(dt: number): void {
    const sub = this.config.substeps
    const h = dt / sub
    for (let s = 0; s < sub; s++) this.substep(h)
    this.time += dt
    this.recordInstruments()
  }

  private substep(dt: number): void {
    const bodies = this.bodies
    const cfg = this.config
    const field = this.gravity.field

    // --- force phase ---
    for (const b of bodies) {
      b.force.set(0, 0)
      b.torque = 0
    }
    for (const e of this.emitters) this.updateEmitter(e, dt)
    if (this.fluid) {
      this.fluid.step(dt, bodies, field)
      this.fluid.applyReactionForces(bodies)
    }
    this.gravity.apply(bodies)
    if (cfg.airDrag !== 0) {
      for (const b of bodies) {
        if (!b.isDynamic() || b.sleeping) continue
        b.force.x -= cfg.airDrag * b.mass * b.vel.x
        b.force.y -= cfg.airDrag * b.mass * b.vel.y
      }
    }
    for (const c of this.constraints) c.applyForces()

    // --- velocity integration ---
    const verlet = cfg.integrator === 'verlet'
    if (verlet) {
      // Velocity Verlet: first half-kick, then the position update with the
      // midpoint velocity (x += (v + a dt/2) dt — what makes it 2nd-order
      // symplectic), then the narrowphase on the NEW positions, then the
      // second half-kick with forces recomputed at the new position, and
      // only then the velocity solve.
      for (const b of bodies) {
        if (!b.isDynamic() || b.sleeping) continue
        b.vel.x += b.force.x * b.invMass * (dt / 2)
        b.vel.y += b.force.y * b.invMass * (dt / 2)
        b.angVel += b.torque * b.invInertia * (dt / 2)
      }
      this.integratePositions(bodies, dt)
      this.buildManifolds(bodies)
      if (cfg.gravityMode === 'nbody') {
        for (const b of bodies) {
          b.force.set(0, 0)
          b.torque = 0
        }
        this.gravity.apply(bodies)
      }
      for (const b of bodies) {
        if (!b.isDynamic() || b.sleeping) continue
        b.vel.x += b.force.x * b.invMass * (dt / 2)
        b.vel.y += b.force.y * b.invMass * (dt / 2)
        b.angVel += b.torque * b.invInertia * (dt / 2)
      }
    } else {
      // Semi-implicit (symplectic) Euler: full kick first, solve against the
      // pre-move geometry (Box2D ordering).
      for (const b of bodies) {
        if (!b.isDynamic() || b.sleeping) continue
        b.vel.x += (b.force.x * b.invMass - b.linearDamping * b.vel.x) * dt
        b.vel.y += (b.force.y * b.invMass - b.linearDamping * b.vel.y) * dt
        b.angVel += (b.torque * b.invInertia - b.angularDamping * b.angVel) * dt
      }
      this.buildManifolds(bodies)
    }

    // --- velocity solve ---
    // Split-impulse: the velocity solve applies restitution only (baumgarte
    // = 0). Penetration recovery happens in the projection pass below, so
    // the two never inject energy into each other.
    const params = {
      iterations: cfg.solverIterations,
      restitutionThreshold: cfg.restitutionThreshold,
      slop: cfg.slop,
      baumgarte: 0,
    }
    for (const m of this.manifolds) {
      if (m.points.length) this.solver.prepare(m)
    }
    // Wake logic: a sleeping body contacted by a strong impact behaves as
    // static (effInvMass = 0) until the impact is large enough to wake it.
    for (const m of this.manifolds) {
      if (!m.points.length) continue
      const aAwake = m.a.isDynamic() && !m.a.sleeping
      const bAwake = m.b.isDynamic() && !m.b.sleeping
      const resting = aAwake ? m.b : m.a
      if (!aAwake && !bAwake) {
        m.points.length = 0 // both dormant: skip
        continue
      }
      if (resting.sleeping) {
        let strong = false
        for (const pt of m.points) {
          if (pt.vn0 < -0.15 || pt.penetration > 0.02) {
            strong = true
            break
          }
        }
        if (strong) resting.wake()
      }
    }
    for (let iter = 0; iter < cfg.solverIterations; iter++) {
      for (const m of this.manifolds) {
        if (m.points.length) this.solver.solve(m, dt, params)
      }
      for (const c of this.constraints) c.solveVelocity(dt)
    }
    this.solver.commit(this.manifolds)

    // Position correction: project bodies out of penetration without
    // touching velocities (no energy injection, unlike a strong Baumgarte
    // bias). Restitution stays exact because the velocity solve already ran.
    {
      // Keep this moderate: a too-aggressive projection can push one body's
      // center into a third body (the correction is per-manifold), which the
      // CCD sweep then cannot recover from. 0.5 resolves stacks in a few
      // steps without violent single-step displacements.
      const percent = 0.5
      for (const m of this.manifolds) {
        if (!m.points.length) continue
        // One projection per manifold at the deepest point: applying it
        // once per contact point would over-correct 2-point (face) contacts.
        let pen = 0
        let pt = m.points[0]
        for (const p of m.points) {
          if (p.penetration > pen) {
            pen = p.penetration
            pt = p
          }
        }
        pen -= cfg.slop
        if (pen <= 0) continue
        const iA = m.a.effInvMass
        const iB = m.b.effInvMass
        const sum = iA + iB
        if (sum < 1e-9) continue
        const corr = (pen * percent) / sum
        m.a.pos.x -= pt.normal.x * corr * iA
        m.a.pos.y -= pt.normal.y * corr * iA
        m.b.pos.x += pt.normal.x * corr * iB
        m.b.pos.y += pt.normal.y * corr * iB
      }
      // Joints: project anchors back onto the constraint (centripetal term).
      for (const c of this.constraints) c.correctPosition(0.5)
    }

    // Contact events (audio / sparks): strongest impulse per manifold.
    for (const m of this.manifolds) {
      if (!m.points.length) continue
      let best = 0
      for (const pt of m.points) best = Math.max(best, pt.normalImpulse)
      if (best > 0.5) {
        const pt = m.points[0]
        const e: ContactEvent = { x: pt.pos.x, y: pt.pos.y, impulse: best, a: m.a, b: m.b }
        this.contactEvents.push(e)
        if (this.onContact) this.onContact(e)
      }
    }
    this.contactEvents.length = 0

    // --- position integration (verlet already moved positions above) ---
    if (!verlet) this.integratePositions(bodies, dt)

    // --- sleeping ---
    this.updateSleep(dt)

    // --- deformables & particles ---
    for (const c of this.cloths) {
      c.setBodies(bodies)
      c.step(dt)
    }
    for (const b of this.blobs) {
      b.setBodies(bodies)
      b.step(dt)
    }
    this.particles.step(dt, bodies, field)
  }

  private buildManifolds(bodies: Body[]): void {
    for (const b of bodies) b.updateAABB()
    this.manifolds.length = 0
    this.broadphase.collect(bodies, (a, b2) => {
      if (bothSleepingSkip(a, b2)) return
      const m = generateManifold(a, b2)
      if (m) this.manifolds.push(m)
    })
  }

  /** Position integration for all awake bodies, with CCD for small fast ones. */
  private integratePositions(bodies: Body[], dt: number): void {
    for (const b of bodies) {
      if (!b.isDynamic() && b.kind !== 'kinematic') continue
      if (b.sleeping) continue
      b.prevPos.copy(b.pos)
      b.prevAngle = b.angle

      const vx = b.vel.x
      const vy = b.vel.y
      if (b.ccd && b.kind === 'dynamic') {
        const dx = vx * dt
        const dy = vy * dt
        const dispLen = Math.hypot(dx, dy)
        const r = b.shape.type === 'circle' ? b.shape.radius : b.worldRadius() * 0.5
        if (dispLen >= r * 0.5) {
          const disp = new Vec2(dx, dy)
          let bestT = 1
          let hitNormal: Vec2 | null = null
          let hitBody: Body | null = null
          for (const o of bodies) {
            if (o === b) continue
            const res = sweepCircle(b.pos, disp, r, o)
            if (res && res.t < bestT) {
              bestT = res.t
              hitNormal = new Vec2(res.nx, res.ny)
              hitBody = o
            }
          }
          if (hitBody && hitNormal) {
            // Place at time of impact; reflect the normal velocity component.
            b.pos.x += dx * bestT
            b.pos.y += dy * bestT
            const vn = b.vel.x * hitNormal.x + b.vel.y * hitNormal.y
            const e = Math.max(b.material.restitution, hitBody.material.restitution)
            if (vn < 0) {
              b.vel.x -= (1 + e) * vn * hitNormal.x
              b.vel.y -= (1 + e) * vn * hitNormal.y
            }
            hitBody.wake()
            b.angle += b.angVel * dt
            continue
          }
        }
        b.pos.x += vx * dt
        b.pos.y += vy * dt
      } else {
        b.pos.x += vx * dt
        b.pos.y += vy * dt
      }
      b.angle += b.angVel * dt
    }
  }

  private updateSleep(dt: number): void {
    const linTol = 0.05
    const angTol = 0.05
    const sleepTime = 0.5
    for (const b of this.bodies) {
      if (!b.isDynamic()) continue
      if (b.sleeping) continue
      const inContact = this.manifolds.some(
        (m) => (m.a === b || m.b === b) && m.points.some((p) => p.penetration > 1e-4),
      )
      const slow =
        b.vel.lenSq() < linTol * linTol * 0.25 &&
        b.angVel * b.angVel < angTol * angTol * 0.25 &&
        (inContact || Math.abs(this.config.gravityY) + Math.abs(this.config.gravityX) < 1e-9 || this.config.gravityMode === 'none')
      if (slow) {
        b.sleepTimer += dt
        if (b.sleepTimer >= sleepTime && this.config.gravityMode !== 'nbody') {
          b.sleeping = true
          b.vel.set(0, 0)
          b.angVel = 0
        }
      } else {
        b.sleepTimer = 0
      }
    }
  }

  // ------------------------------------------------------------ instruments

  private recordInstruments(): void {
    let ke = 0
    let pe = 0
    let px = 0
    let py = 0
    let L = 0
    for (const b of this.bodies) {
      if (!b.isDynamic() && b.kind !== 'kinematic') continue
      const v2 = b.vel.x * b.vel.x + b.vel.y * b.vel.y
      ke += 0.5 * b.mass * v2 + 0.5 * b.inertia * b.angVel * b.angVel
      px += b.mass * b.vel.x
      py += b.mass * b.vel.y
      L += b.inertia * b.angVel
    }
    pe += this.gravity.potentialEnergy(this.bodies)
    for (const c of this.constraints) {
      if (c instanceof SpringConstraint) pe += c.potentialEnergy()
    }
    for (const c of this.cloths) {
      for (const p of c.particles) ke += 0.5 * c.particleMass * (p.vx * p.vx + p.vy * p.vy)
    }
    if (this.fluid) ke += this.fluid.kineticEnergy()
    this.energy = { ke, pe, total: ke + pe, px, py, L, t: this.time }
    this.history.push(this.energy)
    if (this.history.length > this.historyCapacity) this.history.shift()
    this.recordSnapshot()
  }

  private recordSnapshot(): void {
    const dyn = this.bodies.filter((b) => b.isDynamic() || b.kind === 'kinematic')
    if (dyn.length === 0 || dyn.length > 600) return
    const data = new Float32Array(dyn.length * 6)
    const ids: number[] = []
    dyn.forEach((b, i) => {
      ids.push(b.id)
      data[i * 6] = b.pos.x
      data[i * 6 + 1] = b.pos.y
      data[i * 6 + 2] = b.angle
      data[i * 6 + 3] = b.vel.x
      data[i * 6 + 4] = b.vel.y
      data[i * 6 + 5] = b.angVel
    })
    this.snapshots.push({ t: this.time, data, ids })
    if (this.snapshots.length > this.snapshotCapacity) this.snapshots.shift()
  }

  /**
   * Reverse scrub: restore the state recorded `n` ticks ago. The forward
   * future is discarded; playing again re-simulates deterministically from
   * the restored state.
   */
  scrubBack(n: number): void {
    // The newest snapshot is the current state, so unwinding n ticks pops
    // n + 1 entries (the current one plus the n it must replace).
    for (let i = 0; i < n + 1; i++) {
      const s = this.snapshots[this.snapshots.length - 1]
      if (!s) return
      this.snapshots.pop()
      this.restoreSnapshot(s)
    }
  }

  private restoreSnapshot(s: Snapshot): void {
    const byId = new Map<number, Body>()
    for (const b of this.bodies) byId.set(b.id, b)
    s.ids.forEach((id, i) => {
      const b = byId.get(id)
      if (!b) return
      b.pos.set(s.data[i * 6], s.data[i * 6 + 1])
      b.angle = s.data[i * 6 + 2]
      b.vel.set(s.data[i * 6 + 3], s.data[i * 6 + 4])
      b.angVel = s.data[i * 6 + 5]
      b.prevPos.copy(b.pos)
      b.prevAngle = b.angle
      b.wake()
      b.updateAABB()
    })
    if (s.t < this.time) this.time = s.t
    this.solver.clear()
  }

  // --------------------------------------------------------------- emitters

  private updateEmitter(e: Emitter, dt: number): void {
    if (!e.active) return
    if (e.total >= 0 && e.emitted >= e.total) {
      e.active = false
      return
    }
    e.acc += e.rate * dt
    let n = Math.floor(e.acc)
    e.acc -= n
    while (n-- > 0 && (e.total < 0 || e.emitted < e.total)) {
      const px = e.x + this.rng.range(-e.jitter, e.jitter)
      const py = e.y + this.rng.range(-e.jitter, e.jitter)
      const body = new Body(e.shape, e.material)
      body.pos.set(px, py)
      const r = body.shape.type === 'circle' ? body.shape.radius : body.worldRadius() * 0.5
      // Never spawn into occupied space. An overlapping rain column locks
      // into a contact chain that the projection pass pumps apart, ejecting
      // balls with injected energy; gating keeps the stream self-spaced.
      let blocked = false
      for (const o of this.bodies) {
        const or = o.shape.type === 'circle' ? o.shape.radius : o.worldRadius() * 0.5
        if (Math.hypot(o.pos.x - px, o.pos.y - py) < r + or) {
          blocked = true
          break
        }
      }
      if (blocked) {
        e.acc = 0
        break
      }
      body.vel.set(
        e.velBase.x + this.rng.range(-e.velSpread, e.velSpread),
        e.velBase.y + this.rng.range(-e.velSpread, e.velSpread),
      )
      this.addBody(body)
      e.emitted++
    }
  }

  // ------------------------------------------------------------- utilities

  /** Point-in-body test for picking (largest body wins ties). */
  pickPoint(p: Vec2): Body | null {
    let found: Body | null = null
    for (const b of this.bodies) {
      if (this.pointInBody(b, p)) {
        found = b
      }
    }
    return found
  }

  private pointInBody(b: Body, p: Vec2): boolean {
    const s = b.shape
    const cos = Math.cos(-b.angle)
    const sin = Math.sin(-b.angle)
    const dx = p.x - b.pos.x
    const dy = p.y - b.pos.y
    const lx = dx * cos - dy * sin
    const ly = dx * sin + dy * cos
    if (s.type === 'circle') return lx * lx + ly * ly <= s.radius * s.radius
    for (let i = 0; i < s.verts.length; i++) {
      const a = s.verts[i]
      const c = s.verts[(i + 1) % s.verts.length]
      if ((c.x - a.x) * (ly - a.y) - (c.y - a.y) * (lx - a.x) < 0) return false
    }
    return true
  }

  /** Wake every body (e.g. after a gravity change). */
  wakeAll(): void {
    for (const b of this.bodies) b.wake()
  }
}

function bothSleepingSkip(a: Body, b: Body): boolean {
  return a.sleeping && b.sleeping && a.kind !== 'static' && b.kind !== 'static'
}

export { resetBodyIds, circleShape, boxShape }
export type { Shape, Material }
