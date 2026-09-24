import { describe, it, expect } from 'vitest'
import { Vec2 } from '../src/engine/vec'
import { Body } from '../src/engine/body'
import { circleShape, boxShape, hullShape, massProperties } from '../src/engine/shapes'
import { World, DT } from '../src/engine/world'
import { SpringConstraint, DistanceConstraint, RevoluteConstraint } from '../src/engine/constraints'
import { generateManifold } from '../src/engine/narrowphase'
import { Broadphase } from '../src/engine/broadphase'
import { sweepCircle } from '../src/engine/ccd'

// ---------------------------------------------------------------- geometry

describe('shape mass properties', () => {
  it('computes circle mass and I = m r^2 / 2', () => {
    const { mass, inertia } = massProperties(circleShape(0.5), 2)
    expect(mass).toBeCloseTo(2 * Math.PI * 0.25, 6)
    expect(inertia).toBeCloseTo(mass * 0.25 / 2, 6)
  })
  it('computes box inertia I = m w h^2 style fan integral', () => {
    // square of side 1, density 1: I about centroid = m * (1^2 + 1^2)/12 = m/6
    const { mass, inertia } = massProperties(boxShape(1, 1), 1)
    expect(mass).toBeCloseTo(1, 6)
    expect(inertia).toBeCloseTo(1 / 6, 5)
  })
  it('box rotated 90deg has the same inertia', () => {
    const { inertia: i1 } = massProperties(boxShape(2, 0.5), 1)
    const { inertia: i2 } = massProperties(boxShape(0.5, 2), 1)
    expect(i1).toBeCloseTo(i2, 8)
  })
  it('hull of a triangle is convex and centroid-centered', () => {
    const h = hullShape([new Vec2(0, 0), new Vec2(2, 0), new Vec2(0, 2)])
    // verts are centroid-relative, so their mean must be ~0
    const cx = h.verts.reduce((s, v) => s + v.x, 0) / h.verts.length
    const cy = h.verts.reduce((s, v) => s + v.y, 0) / h.verts.length
    expect(Math.hypot(cx, cy)).toBeLessThan(1e-9)
    // area of triangle (0,0),(2,0),(0,2) is 2
    expect(h.area).toBeCloseTo(2, 6)
  })
})

// ----------------------------------------------------------------- kinematics

describe('integration', () => {
  it('free fall matches 1/2 g t^2', () => {
    const w = new World()
    const b = new Body(circleShape(0.2))
    b.pos.set(0, 10)
    w.addBody(b)
    const t = 1.0
    for (let i = 0; i < Math.round(t / DT); i++) w.step(DT)
    const expectFall = 0.5 * 9.81 * t * t
    const actual = 10 - b.pos.y
    expect(Math.abs(actual - expectFall) / expectFall).toBeLessThan(0.02)
  })

  it('energy of a frictionless e=1 bounce stays within 1% over 30 s', () => {
    const w = new World({ restitutionThreshold: 0 })
    // wide floor: the ball drifts 1.5 m/s * 30 s = 45 m sideways, so the
    // ground must extend past that or it rolls off the edge
    const floor = new Body(boxShape(100, 1), { restitution: 1, friction: 0 })
    floor.setKind('static')
    floor.pos.set(0, -0.5)
    w.addBody(floor)
    const b = new Body(circleShape(0.3), { restitution: 1, friction: 0 })
    b.pos.set(0, 5)
    b.vel.set(1.5, 0)
    w.addBody(b)
    const e0 = w.history.length
    void e0
    // initial total energy after first step settles the manifolds
    for (let i = 0; i < 10; i++) w.step(DT)
    const eStart = w.energy.total
    for (let i = 0; i < 30 / DT; i++) w.step(DT)
    const drift = Math.abs(w.energy.total - eStart) / eStart
    expect(drift).toBeLessThan(0.01)
    expect(b.pos.y).toBeGreaterThan(-0.5) // still bouncing, not lost
  })

  it('linear momentum is exactly conserved in collisions', () => {
    const w = new World({ restitutionThreshold: 0 })
    const a = new Body(circleShape(0.2), { restitution: 1, friction: 0, density: 2 })
    a.pos.set(-2, 0)
    a.vel.set(4, 0)
    const b = new Body(circleShape(0.2), { restitution: 1, friction: 0, density: 1 })
    b.pos.set(2, 0)
    w.addBody(a)
    w.addBody(b)
    const p0 = a.mass * a.vel.x + b.mass * b.vel.x
    for (let i = 0; i < 240; i++) w.step(DT)
    const p1 = a.mass * a.vel.x + b.mass * b.vel.x
    expect(Math.abs(p1 - p0)).toBeLessThan(1e-9)
  })

  it('equal-mass e=1 collision swaps velocities (momentum + KE transfer)', () => {
    const w = new World({ restitutionThreshold: 0 })
    const a = new Body(circleShape(0.2), { restitution: 1, friction: 0, density: 1 })
    a.pos.set(-1, 0)
    a.vel.set(4, 0)
    const b = new Body(circleShape(0.2), { restitution: 1, friction: 0, density: 1 })
    b.pos.set(1, 0)
    w.addBody(a)
    w.addBody(b)
    for (let i = 0; i < 120; i++) w.step(DT)
    expect(a.vel.x).toBeLessThan(0.1)
    expect(b.vel.x).toBeGreaterThan(3.5)
  })
})

// ------------------------------------------------------------------- stacks

describe('contact solver', () => {
  it('a box stack of 5 settles without jitter or sinking (T2)', () => {
    const w = new World()
    const floor = new Body(boxShape(10, 1), { friction: 0.5 })
    floor.setKind('static')
    floor.pos.set(0, -0.5)
    w.addBody(floor)
    const boxes: Body[] = []
    for (let i = 0; i < 5; i++) {
      const b = new Body(boxShape(0.8, 0.4), { friction: 0.5, restitution: 0 })
      b.pos.set(0, 0.2 + i * 0.4 + 0.6)
      w.addBody(b)
      boxes.push(b)
    }
    for (let i = 0; i < 120 * 10; i++) w.step(DT)
    const top = boxes[4]
    // top box center: floor top (0) + 4 gaps of 0.4 + half a box = 1.8
    expect(top.pos.y).toBeGreaterThan(1.75)
    expect(top.pos.y).toBeLessThan(1.86)
    expect(top.vel.len()).toBeLessThan(0.01)
    expect(top.angVel).toBeLessThan(0.01)
    // all boxes eventually sleep
    for (const b of boxes) expect(b.sleeping).toBe(true)
  })

  it('friction holds a box on an incline when mu > tan(theta)', () => {
    const w = new World()
    const th = 25 * (Math.PI / 180)
    const plane = new Body(boxShape(6, 0.3), { friction: 0.5 })
    plane.setKind('static')
    plane.pos.set(0, 0.9)
    plane.angle = th
    w.addBody(plane)
    const bx = -1.7
    const by = 0.9 + bx * Math.tan(th) + 0.25 / Math.cos(th)
    const box = new Body(boxShape(0.5, 0.5), { friction: 0.5, restitution: 0 })
    box.pos.set(bx, by)
    box.angle = th
    w.addBody(box)
    for (let i = 0; i < 120 * 3; i++) w.step(DT)
    expect(box.vel.len()).toBeLessThan(0.05)
  })

  it('CCD stops a fast small ball from tunneling a thin wall', () => {
    const w = new World()
    const wall = new Body(boxShape(0.06, 6), { friction: 0.1 })
    wall.setKind('static')
    wall.pos.set(0, 0)
    w.addBody(wall)
    const ball = new Body(circleShape(0.05), { restitution: 0.5, friction: 0 })
    ball.pos.set(-1, 0)
    ball.vel.set(60, 0) // 0.5 m per step at 120 Hz
    w.addBody(ball)
    w.step(DT)
    expect(ball.pos.x).toBeLessThan(-0.2) // hit the wall, did not pass
  })

  it('broadphase finds exactly the AABB-overlapping pairs (vs brute force)', () => {
    const bodies: Body[] = []
    let seed = 1234
    const rnd = () => {
      seed = (seed * 1103515245 + 12345) & 0x7fffffff
      return seed / 0x7fffffff
    }
    for (let i = 0; i < 60; i++) {
      const b = new Body(i % 2 ? circleShape(0.1 + rnd() * 1.5) : boxShape(0.2 + rnd() * 2, 0.2 + rnd() * 2))
      b.pos.set(rnd() * 8 - 4, rnd() * 8 - 4)
      b.angle = rnd() * Math.PI
      b.updateAABB()
      bodies.push(b)
    }
    const treePairs = new Set<string>()
    const bp = new Broadphase(2)
    bp.collect(bodies, (a, b) => {
      treePairs.add(`${a.id}-${b.id}`)
    })
    const brute = new Set<string>()
    for (let i = 0; i < bodies.length; i++) {
      for (let j = i + 1; j < bodies.length; j++) {
        const A = bodies[i].aabb
        const B = bodies[j].aabb
        if (A.minX <= B.maxX && B.minX <= A.maxX && A.minY <= B.maxY && B.minY <= A.maxY) {
          brute.add(`${Math.min(bodies[i].id, bodies[j].id)}-${Math.max(bodies[i].id, bodies[j].id)}`)
        }
      }
    }
    expect(treePairs.size).toBe(brute.size)
    for (const k of brute) expect(treePairs.has(k)).toBe(true)
  })

  it('GJK/EPA penetration matches SAT within 15% for hull polygons', () => {
    const a = new Body(hullShape([new Vec2(-1, -0.5), new Vec2(1, -0.5), new Vec2(1, 0.5), new Vec2(-1, 0.5)]))
    const b = new Body(hullShape([new Vec2(-0.4, -0.25), new Vec2(0.4, -0.25), new Vec2(0.4, 0.25), new Vec2(-0.4, 0.25)]))
    a.pos.set(0, 0)
    b.pos.set(0.8, 0)
    const sat = new Body(boxShape(2, 1))
    sat.pos.set(0, 0)
    const sb = new Body(boxShape(0.8, 0.5))
    sb.pos.set(0.8, 0)
    const mSat = generateManifold(sat, sb)
    const mGjk = generateManifold(a, b)
    expect(mSat).not.toBeNull()
    expect(mGjk).not.toBeNull()
    const penSat = Math.max(...mSat!.points.map((p) => p.penetration))
    const penGjk = mGjk!.points[0].penetration
    expect(Math.abs(penGjk - penSat) / penSat).toBeLessThan(0.15)
  })

  it('swept circle detects hit time against a box', () => {
    const wall = new Body(boxShape(0.2, 4))
    wall.pos.set(1, 0)
    const p = new Vec2(0, 0)
    const disp = new Vec2(1.0, 0)
    const res = sweepCircle(p, disp, 0.1, wall)
    expect(res).not.toBeNull()
    // hit when the center reaches x = 0.8 (inner face 0.9 minus r 0.1)
    expect(res!.t * 1.0).toBeCloseTo(0.8, 5)
  })
})

// ---------------------------------------------------------------- constraints

describe('constraints', () => {
  it('distance joint holds a rod at rest length', () => {
    const w = new World({ gravityMode: 'none' })
    const b = new Body(circleShape(0.2))
    b.pos.set(1, 0)
    b.vel.set(0, 3)
    w.addBody(b)
    w.addConstraint(new DistanceConstraint(null, b, new Vec2(0, 0), new Vec2(0, 0), 1))
    for (let i = 0; i < 240; i++) w.step(DT)
    const d = Math.hypot(b.pos.x, b.pos.y)
    expect(Math.abs(d - 1)).toBeLessThan(0.02)
  })

  it('spring-mass period matches 2*pi*sqrt(m/k) within 10% (T6)', () => {
    const w = new World({ gravityMode: 'none' })
    const k = 100
    const m = 1
    const b = new Body(boxShape(0.5, 0.5), { density: 1 })
    b.material.density = m / 0.25
    b.recomputeMass()
    b.pos.set(0.5, 0) // start displaced
    w.addBody(b)
    w.addConstraint(new SpringConstraint(null, b, new Vec2(0, 0), new Vec2(0, 0), k, 0))
    const T = 2 * Math.PI * Math.sqrt(b.mass / k)
    // find two consecutive zero-crossings of x
    let prevX = b.pos.x
    let cross1 = -1
    for (let i = 0; i < 120 * (4 * T); i++) {
      w.step(DT)
      const x = b.pos.x
      if (prevX < 0 && x >= 0) {
        if (cross1 >= 0) {
          const measured = (i - cross1) * DT
          expect(Math.abs(measured - T) / T).toBeLessThan(0.1)
          break
        }
        cross1 = i
      }
      prevX = x
    }
    expect(cross1).toBeGreaterThan(0)
  })

  it('damped spring settles (no perpetual oscillation)', () => {
    const w = new World({ gravityMode: 'none' })
    const b = new Body(boxShape(0.5, 0.5), { density: 4 })
    b.pos.set(0.5, 0)
    w.addBody(b)
    w.addConstraint(new SpringConstraint(null, b, new Vec2(0, 0), new Vec2(0, 0), 50, 3))
    for (let i = 0; i < 120 * 8; i++) w.step(DT)
    expect(b.vel.len()).toBeLessThan(0.05)
    expect(Math.abs(b.pos.x)).toBeLessThan(0.05)
  })

  it('revolute joint with motor spins a body (T6)', () => {
    const w = new World({ gravityMode: 'none' })
    const b = new Body(boxShape(0.2, 1), { density: 1 })
    b.pos.set(0, 0.5)
    w.addBody(b)
    // pin the bottom end of the rod at the world origin
    const pin = new RevoluteConstraint(null, b, new Vec2(0, 0), new Vec2(0, -0.5))
    pin.motorOn = true
    pin.motorSpeed = 2
    pin.motorForce = 50
    w.addConstraint(pin)
    for (let i = 0; i < 120 * 3; i++) w.step(DT)
    expect(b.angVel).toBeGreaterThan(1.5)
    // pivot stays fixed
    const pivot = b.localToWorld(new Vec2(0, -0.5))
    expect(pivot.len()).toBeLessThan(0.02)
  })

  it('angle limits hold a pendulum within bounds', () => {
    const w = new World()
    const b = new Body(circleShape(0.2), { density: 2 })
    b.pos.set(0.8, -1.2) // start pushed against the limit side
    w.addBody(b)
    const pin = new RevoluteConstraint(null, b, new Vec2(0, 0), new Vec2(0, 0))
    pin.angleLimit = [-Math.PI / 3, Math.PI / 3]
    pin.limitForce = 200
    w.addConstraint(pin)
    for (let i = 0; i < 120 * 5; i++) w.step(DT)
    const ang = Math.atan2(b.pos.x, -b.pos.y)
    expect(Math.abs(ang)).toBeLessThanOrEqual(Math.PI / 3 + 0.05)
  })
})

// ------------------------------------------------------------------- gravity

describe('N-body gravity', () => {
  it('circular orbit is closed and non-decaying for 25 years (T9)', () => {
    const w = new World({ gravityMode: 'nbody', substeps: 8, integrator: 'verlet' })
    w.syncGravity()
    const sun = new Body(circleShape(0.2), { density: 1 })
    sun.setKind('static')
    sun.userData = { gravMass: 1 }
    w.addBody(sun)
    const p = new Body(circleShape(0.02), { density: 1 })
    p.userData = { gravMass: 3e-6 }
    p.pos.set(1, 0)
    p.vel.set(0, 2 * Math.PI) // circular at 1 AU for M=1
    w.addBody(p)
    const radii: number[] = []
    for (let i = 0; i < 25 * 120; i++) {
      w.step(DT)
      if (i % 120 === 0) radii.push(p.pos.len())
    }
    const min = Math.min(...radii)
    const max = Math.max(...radii)
    // orbit stays within 0.5% of 1 AU over 25 years
    expect(min).toBeGreaterThan(0.995)
    expect(max).toBeLessThan(1.005)
  })

  it('eccentric orbit reaches perihelion and aphelion (Kepler)', () => {
    const w = new World({ gravityMode: 'nbody', substeps: 8 })
    w.syncGravity()
    const sun = new Body(circleShape(0.2), { density: 1 })
    sun.setKind('static')
    sun.userData = { gravMass: 1 }
    w.addBody(sun)
    const a = 2
    const e = 0.6
    const peri = a * (1 - e)
    const vPeri = (2 * Math.PI) / Math.sqrt(a) * Math.sqrt((1 + e) / (1 - e))
    const c = new Body(circleShape(0.02), { density: 1 })
    c.userData = { gravMass: 1e-9 }
    c.pos.set(peri, 0)
    c.vel.set(0, vPeri)
    w.addBody(c)
    let min = Infinity
    let max = -Infinity
    for (let i = 0; i < 5 * 120; i++) {
      w.step(DT)
      const r = c.pos.len()
      min = Math.min(min, r)
      max = Math.max(max, r)
    }
    expect(min).toBeCloseTo(a * (1 - e), 1)
    expect(max).toBeCloseTo(a * (1 + e), 1)
  })

  it('semi-implicit Euler drifts orbit energy; velocity Verlet does not', () => {
    const mk = (integrator: 'euler' | 'verlet') => {
      const w = new World({ gravityMode: 'nbody', substeps: 4, integrator })
      w.syncGravity()
      const sun = new Body(circleShape(0.2), { density: 1 })
      sun.setKind('static')
      sun.userData = { gravMass: 1 }
      w.addBody(sun)
      const p = new Body(circleShape(0.02), { density: 1 })
      p.userData = { gravMass: 3e-6 }
      p.pos.set(1, 0)
      p.vel.set(0, 2 * Math.PI)
      w.addBody(p)
      return { w, p }
    }
    const run = ({ w, p }: { w: World; p: Body }) => {
      for (let i = 0; i < 20 * 120; i++) w.step(DT)
      return p.pos.len()
    }
    const rVerlet = run(mk('verlet'))
    const rEuler = run(mk('euler'))
    expect(Math.abs(rVerlet - 1)).toBeLessThan(0.01)
    // plain Euler at this step count shows visible radial drift
    expect(Math.abs(rEuler - 1)).toBeGreaterThan(Math.abs(rVerlet - 1))
  })
})

// ----------------------------------------------------------------- determinism

describe('determinism', () => {
  it('identical seeds replay identically', () => {
    const build = () => {
      const w = new World({ seed: 42 })
      for (let i = 0; i < 30; i++) {
        const b = new Body(i % 2 ? circleShape(0.1 + (i % 5) * 0.05) : boxShape(0.3, 0.3))
        b.pos.set(Math.sin(i) * 3, 4 + (i % 3))
        b.vel.set(Math.cos(i) * 2, 0)
        w.addBody(b)
      }
      const floor = new Body(boxShape(30, 1), { friction: 0.3 })
      floor.setKind('static')
      floor.pos.set(0, -0.5)
      w.addBody(floor)
      return w
    }
    const w1 = build()
    const w2 = build()
    for (let i = 0; i < 600; i++) {
      w1.step(DT)
      w2.step(DT)
    }
    for (let i = 0; i < w1.bodies.length; i++) {
      expect(w1.bodies[i].pos.x).toBe(w2.bodies[i].pos.x)
      expect(w1.bodies[i].pos.y).toBe(w2.bodies[i].pos.y)
      expect(w1.bodies[i].vel.x).toBe(w2.bodies[i].vel.x)
      expect(w1.bodies[i].vel.y).toBe(w2.bodies[i].vel.y)
      expect(w1.bodies[i].angle).toBe(w2.bodies[i].angle)
    }
  })
})

// ----------------------------------------------------- render interpolation

describe('render interpolation pose', () => {
  it('keeps static bodies\' prevPos/prevAngle in sync so the renderer does not lerp them from the origin', () => {
    const w = new World()
    const bar = new Body(boxShape(2, 0.1))
    bar.setKind('static')
    bar.pos.set(0, 2.2)
    bar.angle = 0.3
    w.addBody(bar)
    const ball = new Body(circleShape(0.1))
    ball.pos.set(0, 1)
    w.addBody(ball)
    for (let i = 0; i < 5; i++) w.step(DT)
    // regression: static bodies were skipped by position integration, so their
    // prevPos stayed at the constructor's (0,0) and the canvas trembled
    expect(bar.prevPos.x).toBeCloseTo(bar.pos.x, 9)
    expect(bar.prevPos.y).toBeCloseTo(bar.pos.y, 9)
    expect(bar.prevAngle).toBeCloseTo(bar.angle, 9)
  })
})
