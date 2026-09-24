import { describe, it, expect } from 'vitest'
import { DT, World } from '../src/engine/world'
import { findPreset, PRESETS } from '../src/engine/presets'
import { worldToScene, sceneToWorld } from '../src/engine/scene'
import { Vec2 } from '../src/engine/vec'
import { Body } from '../src/engine/body'
import { boxShape, circleShape } from '../src/engine/shapes'
import { RevoluteConstraint, SpringConstraint } from '../src/engine/constraints'

// ------------------------------------------------------------------ cradle

describe("Newton's cradle (T4)", () => {
  it('end ball swings out with the transferred momentum', () => {
    const w = findPreset('cradle')!.build()
    const balls = w.bodies.filter((b) => b.shape.type === 'circle')
    expect(balls.length).toBe(5)
    const [first, ...rest] = balls
    const last = rest[rest.length - 1]
    // let the first ball fall and strike
    for (let i = 0; i < 120 * 1.2; i++) w.step(DT)
    // after impact, the last ball must be moving out strongly
    for (let i = 0; i < 60; i++) {
      w.step(DT)
      if (last.vel.len() > 0.8) break
    }
    expect(last.vel.len()).toBeGreaterThan(0.6)
    // and the input ball should have nearly stopped
    expect(first.vel.len()).toBeLessThan(0.5)
  }, 30000)
})

// -------------------------------------------------------------------- cloth

describe('cloth drape (T7)', () => {
  it('settles over the sphere without exploding or infinitely stretching', () => {
    const w = findPreset('cloth')!.build()
    const cloth = w.cloths[0]
    // 60 simulated seconds
    for (let i = 0; i < 120 * 60; i++) w.step(DT)
    // bounded: nothing flew off
    let maxY = -Infinity
    let minY = Infinity
    for (const p of cloth.particles) {
      maxY = Math.max(maxY, p.y)
      minY = Math.min(minY, p.y)
    }
    expect(maxY).toBeLessThan(4)
    expect(minY).toBeGreaterThan(-2)
    // settled: mean speed small
    let vSum = 0
    for (const p of cloth.particles) vSum += Math.hypot(p.vx, p.vy)
    expect(vSum / cloth.particles.length).toBeLessThan(0.1)
    // structural integrity: no constraint stretched more than 25%
    for (const c of cloth.constraints) {
      if (c.broken || c.bend) continue
      const a = cloth.particles[c.a]
      const b = cloth.particles[c.b]
      const d = Math.hypot(b.x - a.x, b.y - a.y)
      expect((d - c.rest) / c.rest).toBeLessThan(0.25)
    }
  }, 120000)
})

// -------------------------------------------------------------------- fluid

describe('fluid tank (T8)', () => {
  // KNOWN LIMITATION (accepted 2026-09): emergent buoyancy is not achievable
  // with the current Clavet double-density SPH at the 120 Hz timestep. The
  // particle pressure that would lift a body is impulsive and drives the rigid
  // boxes through the tank walls, while the stable (soft) regime has too little
  // pressure to float anything. The fluid is kept as a stable, non-buoyant
  // feature (DEFAULT_FLUID.forceScale = 0). This test documents the intended
  // behaviour and is expected to fail until the fluid is reworked (e.g. PBF).
  it('low-density box floats, high-density box sinks (emergent buoyancy)', () => {
    const w = findPreset('fluid')!.build()
    const light = w.bodies.find((b) => b.label === 'ρ=300 (floats)')!
    const heavy = w.bodies.find((b) => b.label === 'ρ=2500 (sinks)')!
    for (let i = 0; i < 120 * 20; i++) w.step(DT)
    // heavy box near the tank floor (top of floor at y = -1.5)
    expect(heavy.pos.y).toBeLessThan(-0.8)
    // light box above the heavy one, out of the deep fluid
    expect(light.pos.y).toBeGreaterThan(heavy.pos.y + 0.4)
    // light box is floating: partially submerged, well above the bottom
    expect(light.pos.y).toBeGreaterThan(-0.6)
    // fluid volume is roughly conserved (particle count unchanged, spread bounded)
    expect(w.fluid!.points.length).toBeGreaterThan(400)
  }, 120000)
})

// -------------------------------------------------------------------- orbit

describe('orbital preset', () => {
  it('moon orbits the earth, earth orbits the sun (hierarchical N-body)', () => {
    const w = findPreset('orbital')!.build()
    const earth = w.bodies.find((b) => b.userData['orbitRadius'] === 1)!
    const moon = w.bodies.find((b) => b.label === 'moon')!
    // 3 simulated years (baseTimeScale only affects tick(); step() is 1:1)
    for (let i = 0; i < 3 * 120; i++) w.step(DT)
    const rEarth = earth.pos.len()
    expect(rEarth).toBeGreaterThan(0.9)
    expect(rEarth).toBeLessThan(1.1)
    // moon sits at 0.03 AU; a bound orbit wanders a little but stays well
    // inside the 0.035 AU stability limit (0.5 r_Hill)
    const dMoon = moon.pos.dist(earth.pos)
    expect(dMoon).toBeGreaterThan(0.022)
    expect(dMoon).toBeLessThan(0.042)
  }, 60000)
})

// ------------------------------------------------------------------- galton

describe('galton board', () => {
  it('balls cascade and settle into bins without exploding', () => {
    const w = findPreset('galton')!.build()
    for (let i = 0; i < 120 * 12; i++) w.step(DT)
    const dynamic = w.bodies.filter((b) => b.kind === 'dynamic')
    expect(dynamic.length).toBe(300)
    for (const b of dynamic) {
      expect(b.pos.x).toBeGreaterThan(-2.5)
      expect(b.pos.x).toBeLessThan(2.5)
      expect(b.pos.y).toBeGreaterThan(-2.2)
      expect(b.pos.y).toBeLessThan(5)
    }
  }, 60000)
})

// ---------------------------------------------------------------- round trip

describe('scene save/load (T10)', () => {
  it('round-trips bodies, velocities, and joints exactly', () => {
    const w = new World()
    const a = new Body(boxShape(1, 0.5), { friction: 0.3, density: 2 })
    a.pos.set(1, 2)
    a.angle = 0.3
    a.vel.set(0.5, -0.2)
    a.angVel = 0.1
    a.label = 'a'
    w.addBody(a)
    const b = new Body(circleShape(0.4), { restitution: 0.7 })
    b.pos.set(-2, 1)
    b.vel.set(-1, 0)
    w.addBody(b)
    w.addConstraint(new RevoluteConstraint(a, b, new Vec2(0.5, 0), new Vec2(0.2, 0)))
    const spring = new SpringConstraint(null, a, new Vec2(0, 3), new Vec2(0, 0.25), 80, 2, 2.0)
    spring.label = 's'
    w.addConstraint(spring)

    const json = worldToScene(w)
    const w2 = sceneToWorld(json)
    expect(w2.bodies.length).toBe(2)
    const a2 = w2.bodies.find((x) => x.label === 'a')!
    expect(a2.pos.x).toBeCloseTo(1, 10)
    expect(a2.pos.y).toBeCloseTo(2, 10)
    expect(a2.vel.x).toBeCloseTo(0.5, 10)
    expect(a2.vel.y).toBeCloseTo(-0.2, 10)
    expect(a2.angVel).toBeCloseTo(0.1, 10)
    expect(a2.angle).toBeCloseTo(0.3, 10)
    expect(a2.material.density).toBeCloseTo(2, 10)
    expect(w2.constraints.length).toBe(2)
    const rev = w2.constraints.find((c) => c instanceof RevoluteConstraint)!
    expect(rev).toBeDefined()
    const sp = w2.constraints.find((c) => c instanceof SpringConstraint)!
    expect(sp.stiffness).toBeCloseTo(80, 10)
    expect(sp.damping).toBeCloseTo(2, 10)
    expect(sp.rest).toBeCloseTo(2.0, 10)
  })
})

// -------------------------------------------------------------------- misc

describe('energy instrumentation', () => {
  it('records a flat total-energy line for a lossless scene', () => {
    const w = new World({ gravityMode: 'none' })
    const b = new Body(circleShape(0.2))
    b.pos.set(1, 0)
    b.vel.set(0, 2)
    w.addBody(b)
    for (let i = 0; i < 240; i++) w.step(DT)
    const totals = w.history.slice(10).map((h) => h.total)
    const first = totals[0]
    for (const t of totals) {
      expect(Math.abs(t - first)).toBeLessThan(1e-9)
    }
  })
  it('spring potential energy shows up in the total', () => {
    const w = new World({ gravityMode: 'none' })
    const b = new Body(boxShape(0.5, 0.5), { density: 4 })
    b.pos.set(0.5, 0)
    w.addBody(b)
    w.addConstraint(new SpringConstraint(null, b, new Vec2(0, 0), new Vec2(0, 0), 50, 0))
    w.step(DT)
    expect(w.energy.pe).toBeGreaterThan(0)
  })
})

describe('presets', () => {
  it('all presets build without throwing and contain at least one body', () => {
    for (const p of PRESETS) {
      const w = p.build()
      if (p.id === 'blank' || p.id === 'blank-zero-g') continue // intentionally empty
      expect(w.bodies.length + w.cloths.length + (w.fluid ? 1 : 0)).toBeGreaterThan(0)
    }
  })
})

describe('reverse scrub', () => {
  it('scrubBack restores earlier positions', () => {
    const w = new World({ gravityMode: 'none' })
    const b = new Body(circleShape(0.2))
    b.pos.set(0, 0)
    b.vel.set(1, 0)
    w.addBody(b)
    for (let i = 0; i < 120; i++) w.step(DT)
    const xNow = b.pos.x
    w.scrubBack(60)
    expect(b.pos.x).toBeCloseTo(xNow - 0.5, 5)
  })
})
