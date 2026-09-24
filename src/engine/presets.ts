import { Vec2 } from './vec'
import { Body } from './body'
import { circleShape, boxShape, hullShape } from './shapes'
import { World, Emitter } from './world'
import {
  DistanceConstraint,
  RopeConstraint,
  RevoluteConstraint,
  SpringConstraint,
  WeldConstraint,
} from './constraints'
import { Cloth } from './softbody'
import { Fluid } from './fluid'

export interface Preset {
  id: string
  name: string
  description: string
  build: () => World
}

const floorMat = { restitution: 0.1, friction: 0.6, density: 1, color: '#3b4a6b', glow: false }
const ballMat = { restitution: 0.9, friction: 0.1, density: 2, color: '#e8e6e3', glow: true }
const woodMat = { restitution: 0.2, friction: 0.4, density: 1, color: '#c98d5a', glow: false }

function ground(w: World, x = 0, y = -1, halfW = 30): void {
  const g = new Body(boxShape(halfW * 2, 0.5), { ...floorMat, density: 1 })
  g.setKind('static')
  g.pos.set(x, y)
  w.addBody(g)
}

// 1 --------------------------------------------------------------- cradle

function newtonsCradle(): World {
  const w = new World()
  ground(w, 0, -0.5, 4)
  const n = 5
  const r = 0.15
  const barY = 2.2
  const rodLen = 1.4
  // A tiny gap between balls: with exactly-touching balls the solver sees a
  // single simultaneous 5-body collision and splits the momentum across all
  // of them. Real cradles transfer one ball because the impact propagates as
  // a compression wave; a sub-pixel gap makes each ball strike the next on a
  // separate step, reproducing that one-in-one-out behaviour.
  const gap = 0.02
  const spacing = 2 * r + gap
  const x0 = -((n - 1) * spacing) / 2
  // top bar (static)
  const bar = new Body(boxShape(n * 2.6 * r, 0.08), { ...floorMat })
  bar.setKind('static')
  bar.pos.set(0, barY)
  w.addBody(bar)
  for (let i = 0; i < n; i++) {
    const x = x0 + i * spacing
    // restitution 1: with e < 1 the "one ball out" transfer is incomplete
    // and the striking ball keeps most of its speed
    const b = new Body(circleShape(r), { ...ballMat, restitution: 1 })
    if (i === 0) {
      // pull the first ball aside by 30 degrees
      const th = 0.52
      const dx = -Math.sin(th)
      const dy = -Math.cos(th)
      b.pos.set(x + rodLen * dx, barY + rodLen * dy - r)
    } else {
      b.pos.set(x, barY - rodLen - r)
    }
    w.addBody(b)
    // rod from a world pivot on the bar to the top of the ball
    const c = new DistanceConstraint(null, b, new Vec2(x, barY), new Vec2(0, r), rodLen)
    c.label = `rod ${i + 1}`
    w.addConstraint(c)
  }
  return w
}

// 2 ------------------------------------------------------- double pendulum

function doublePendulum(): World {
  const w = new World()
  const L1 = 0.9
  const L2 = 0.7
  const mk = (x0: number, a0: number, a1: number, label: string) => {
    const anchorY = 2.4
    const a1x = x0 + L1 * Math.sin(a0)
    const a1y = anchorY - L1 * Math.cos(a0)
    const a2x = a1x + L2 * Math.sin(a1)
    const a2y = a1y - L2 * Math.cos(a1)
    // Link angles: box local -Y axis points along the swing direction
    // (sin a, -cos a) when angle = a, so link1 local (0, L1/2) sits at the
    // anchor end and (0, -L1/2) at the bob end.
    const l1 = new Body(boxShape(0.05, L1), { ...woodMat, density: 3 })
    l1.pos.set((x0 + a1x) / 2, (anchorY + a1y) / 2)
    l1.angle = a0
    w.addBody(l1)
    const l2 = new Body(boxShape(0.05, L2), { ...woodMat, density: 3 })
    l2.pos.set((a1x + a2x) / 2, (a1y + a2y) / 2)
    l2.angle = a1
    w.addBody(l2)
    // bobs
    const bob1 = new Body(circleShape(0.09), { ...ballMat, density: 4 })
    bob1.pos.set(a1x, a1y)
    w.addBody(bob1)
    const bob2 = new Body(circleShape(0.09), { ...ballMat, density: 4 })
    bob2.pos.set(a2x, a2y)
    w.addBody(bob2)

    const pinTop = new RevoluteConstraint(null, l1, new Vec2(x0, anchorY), new Vec2(0, L1 / 2))
    pinTop.label = `${label} top pin`
    w.addConstraint(pinTop)
    const pinMid = new RevoluteConstraint(l1, bob1, new Vec2(0, -L1 / 2), new Vec2(0, 0))
    pinMid.label = `${label} mid pin`
    w.addConstraint(pinMid)
    const weld12 = new WeldConstraint(bob1, l2, new Vec2(0, 0), new Vec2(0, L2 / 2))
    weld12.label = `${label} link weld`
    w.addConstraint(weld12)
    const pin2 = new RevoluteConstraint(l2, bob2, new Vec2(0, -L2 / 2), new Vec2(0, 0))
    pin2.label = `${label} bottom pin`
    w.addConstraint(pin2)
  }
  // Two nearly identical pendulums, 0.2 deg apart — watch chaos diverge.
  mk(-1.2, 2.4, 2.4, 'A')
  mk(1.2, 2.4 + 0.2 * (Math.PI / 180), 2.4 + 0.2 * (Math.PI / 180), 'B')
  return w
}

// 3 ------------------------------------------------------------ projectile

function projectileRange(): World {
  const w = new World()
  ground(w, 0, -0.25, 20)
  const b = new Body(circleShape(0.12), { ...ballMat, friction: 0, density: 1 })
  const v0 = 12
  const th = (40 * Math.PI) / 180
  b.pos.set(-6, 1.6)
  b.vel.set(v0 * Math.cos(th), v0 * Math.sin(th))
  b.label = `projectile (v₀=${v0} m/s, 40°)`
  b.userData = { v0, angleDeg: 40, trackMaxHeight: true }
  w.addBody(b)
  return w
}

// 4 ------------------------------------------------------ inclined friction

function inclinedPlane(): World {
  const w = new World()
  const angle = 25 * (Math.PI) / 180
  const plane = new Body(boxShape(6, 0.3), { ...floorMat, friction: 0.5 })
  plane.setKind('static')
  plane.pos.set(0, 0.9)
  plane.angle = angle
  plane.label = 'plane (θ=25°, mu=0.5 — just above tanθ=0.466: it holds)'
  w.addBody(plane)
  // rest position: on the plane surface near x = -1.7
  const bx = -1.7
  const by = 0.9 + bx * Math.tan(angle) + 0.25 / Math.cos(angle)
  const box = new Body(boxShape(0.5, 0.5), { ...woodMat, friction: 0.5 })
  box.pos.set(bx, by)
  box.angle = angle
  box.label = 'slider'
  w.addBody(box)
  ground(w, 0, -1.6, 8)
  return w
}

// 5 -------------------------------------------------------- spring oscillator

function springOscillator(): World {
  const w = new World()
  const k = 40
  const m = 1
  const mass = new Body(boxShape(0.5, 0.5), { ...woodMat, density: 1 })
  mass.material.density = m / 0.25
  mass.recomputeMass()
  mass.pos.set(0, -0.8)
  mass.label = `mass (T = 2π√(m/k) = ${(2 * Math.PI * Math.sqrt(mass.mass / k)).toFixed(3)} s)`
  w.addBody(mass)
  const s = new SpringConstraint(null, mass, new Vec2(0, 1.6), new Vec2(0, 0.25), k, 0.4, 2.4)
  s.label = 'spring k=40'
  w.addConstraint(s)
  return w
}

// 6 ------------------------------------------------------------- collision lab

function collisionLab(): World {
  const w = new World()
  ground(w, 0, -0.5, 8)
  const floorTop = -0.25
  const a = new Body(circleShape(0.2), { ...ballMat, restitution: 1, friction: 0, density: 2 })
  a.pos.set(-3, floorTop + 0.2)
  a.vel.set(4, 0)
  a.label = 'moving (m=2)'
  w.addBody(a)
  const b = new Body(circleShape(0.2), { ...ballMat, restitution: 1, friction: 0, density: 1 })
  b.pos.set(3, floorTop + 0.2)
  b.label = 'stationary (m=1)'
  w.addBody(b)
  return w
}

// 7 ------------------------------------------------------------ orbital mechanics

function orbitalMechanics(): World {
  const w = new World({
    gravityMode: 'nbody',
    baseTimeScale: 2, // 1x = 2 simulated years per second
    substeps: 16, // the moon orbits in 0.16 yr; keep > ~250 steps/orbit
    integrator: 'verlet',
    seed: 7,
  })
  w.syncGravity()
  // Units: AU, years, solar masses (G = 4π², so M=1 & a=1AU -> P=1yr).
  const sun = new Body(circleShape(0.25), { restitution: 0, friction: 0, density: 1, color: '#ffd54f', glow: true })
  sun.setKind('static')
  sun.userData = { gravMass: 1, isSun: true }
  w.addBody(sun)

  // Earth is ~1000x real Earth mass so the moon at a visible 0.03 AU stays
  // well inside the Hill sphere (r_Hill = (M/3)^(1/3) = 0.1; the moon at
  // 0.3 r_Hill sees only ~28% solar pull relative to Earth's, so the orbit
  // is not systematically pumped). Moon period 0.095 yr ~ 180 substeps.
  const earthM = 3e-3
  // Radii are collision hulls, not to scale: earth + moon must fit inside
  // the 0.03 AU moon orbit or the contact solver ejects the moon.
  const earth = new Body(circleShape(0.012), { restitution: 0, friction: 0, density: 1, color: '#5b8def', glow: false })
  earth.userData = { gravMass: earthM, orbitRadius: 1 }
  earth.pos.set(1, 0)
  earth.vel.set(0, 2 * Math.PI) // circular orbit v = √(GM/a) = 2π at a=1, M=1
  w.addBody(earth)

  const moon = new Body(circleShape(0.006), { restitution: 0, friction: 0, density: 1, color: '#cfd4dc' })
  const moonR = 0.03 // AU from Earth
  moon.userData = { gravMass: 2.7e-8 }
  moon.pos.set(1 + moonR, 0)
  // Moon's circular speed around Earth (GM_earth = 4π²·earthM):
  // v = √(GM/r) = 2π√(M/r). (2π√(M/r³) is the angular velocity ω; it only
  // equals the linear speed when r = 1, which is why Earth happened to work.)
  const vMoon = 2 * Math.PI * Math.sqrt(earthM / moonR)
  moon.vel.set(0, 2 * Math.PI + vMoon) // Earth's velocity + local circular
  moon.label = 'moon'
  w.addBody(moon)

  // A comet on an eccentric ellipse (e = 0.6, a = 2 AU).
  const comet = new Body(circleShape(0.02), { restitution: 0, friction: 0, density: 1, color: '#b39ddb' })
  const aC = 2
  const eC = 0.6
  const peri = aC * (1 - eC)
  // vis-viva at perihelion for M_sun = 1: v = 2π/sqrt(a) * sqrt((1+e)/(1-e))
  const vPeri = (2 * Math.PI) / Math.sqrt(aC) * Math.sqrt((1 + eC) / (1 - eC))
  comet.userData = { gravMass: 1e-9, orbitRadius: aC, eccentricity: eC }
  comet.pos.set(peri, 0)
  comet.vel.set(0, vPeri)
  comet.label = 'comet (e=0.6)'
  w.addBody(comet)
  return w
}

// 8 ----------------------------------------------------------------- galton

function galtonBoard(): World {
  const w = new World()
  const rows = 9
  const topY = 4.2
  // The ball must fit through the V between two pegs, so
  // 2*ballR < gap - 2*pegR. (With the old sizes the ball was wider than
  // the V and every row trapped it on the pegs.)
  const pegR = 0.035
  const gap = 0.18
  const ballR = 0.04
  const bottomY = -1.2
  // pegs: a centered triangle. Row r holds (rows - r) pegs, each row
  // centered on x = 0. (A skewed row layout wedges balls in the gaps and
  // the position projection pumps them apart.)
  for (let r = 0; r < rows; r++) {
    const y = topY - r * (gap * 0.9)
    const count = rows - r
    for (let i = 0; i < count; i++) {
      const x = -((count - 1) / 2) * gap + i * gap
      const p = new Body(circleShape(pegR), { ...floorMat, friction: 0.05, restitution: 0.4 })
      p.setKind('static')
      p.pos.set(x, y)
      w.addBody(p)
    }
  }
  // walls: extend above the emitter (balls bouncing off the top peg row can
  // reach y > 4.5) and below the floor top so no side gap remains
  for (const sx of [-1, 1]) {
    const wall = new Body(boxShape(0.12, 6.8), { ...floorMat })
    wall.setKind('static')
    wall.pos.set(sx * (rows * gap * 0.75 + 0.3), 1.5)
    w.addBody(wall)
  }
  // trays (bins): tall enough for the final pile (300 balls / 11 bins is a
  // ~2 m stack) so balls never cascade out over the rims
  const binCount = 11
  const binW = gap
  const binH = 2.4
  const binCenterY = -1.8 + binH / 2 // bottom embedded in the floor top (-1.72)
  for (let i = 0; i < binCount; i++) {
    const x = -((binCount - 1) / 2) * binW + i * binW
    const left = new Body(boxShape(0.03, binH), { ...floorMat })
    left.setKind('static')
    left.pos.set(x - binW / 2, binCenterY)
    w.addBody(left)
    const right = new Body(boxShape(0.03, binH), { ...floorMat })
    right.setKind('static')
    right.pos.set(x + binW / 2, binCenterY)
    w.addBody(right)
  }
  const floor = new Body(boxShape(4, 0.2), { ...floorMat })
  floor.setKind('static')
  floor.pos.set(0, bottomY - 0.62)
  w.addBody(floor)
  // emitter: rain of small bouncy balls
  const e: Emitter = {
    x: 0,
    y: topY + 0.4,
    shape: circleShape(ballR),
    material: { ...ballMat, restitution: 0.5, friction: 0.02 },
    rate: 60,
    total: 300,
    // The stream is gated by spawn clearance, not rate: the ball below
    // must clear 2*ballR before the next one spawns, so the launch speed
    // sets the true throughput. 300 balls need > 25/s.
    velBase: new Vec2(0, -6),
    velSpread: 0.3,
    jitter: 0.05,
    elapsed: 0,
    acc: 0,
    emitted: 0,
    active: true,
  }
  w.addEmitter(e)
  return w
}

// 9 ---------------------------------------------------------------- cloth drape

function clothDrape(): World {
  const w = new World()
  ground(w, 0, -1.6, 6)
  const sphere = new Body(circleShape(0.9), { ...woodMat, friction: 0.2 })
  sphere.setKind('static')
  sphere.pos.set(0, -0.7)
  sphere.label = 'sphere'
  w.addBody(sphere)
  const cols = 21
  const rows = 13
  const spacing = 0.13
  const cloth = new Cloth(cols, rows, spacing, '#e8c26a', 0.5)
  // hang the cloth over the sphere, pinned at the two top corners
  const cx = -((cols - 1) / 2) * spacing
  const cy = 1.6
  cloth.translate(cx, cy)
  cloth.pin(0, cx, cy)
  cloth.pin(cols - 1, cx + (cols - 1) * spacing, cy)
  w.addCloth(cloth)
  return w
}

// 10 ------------------------------------------------------------- fluid tank

function fluidTank(): World {
  const w = new World()
  const tankW = 3
  const tankH = 2.4
  const t = 0.15
  const mat = { ...floorMat, density: 1 }
  const mkWall = (x: number, y: number, hw: number, hh: number) => {
    const b = new Body(boxShape(hw, hh), mat)
    b.setKind('static')
    b.pos.set(x, y)
    w.addBody(b)
  }
  mkWall(0, -tankH / 2 - t / 2, tankW / 2 + t, t) // floor
  mkWall(-tankW / 2 - t / 2, 0, t, tankH + t) // left
  mkWall(tankW / 2 + t / 2, 0, t, tankH + t) // right
  const fluid = new Fluid()
  fluid.spawnDisk(0, 0.4, 1.0, 420)
  w.setFluid(fluid)
  // low-density box floats, high-density box sinks
  const light = new Body(boxShape(0.7, 0.4), { restitution: 0.05, friction: 0.3, density: 300, color: '#81c784' })
  light.pos.set(-0.7, 0.9)
  light.label = 'ρ=300 (floats)'
  w.addBody(light)
  const heavy = new Body(boxShape(0.7, 0.4), { restitution: 0.05, friction: 0.3, density: 2500, color: '#e57373' })
  heavy.pos.set(0.7, 0.9)
  heavy.label = 'ρ=2500 (sinks)'
  w.addBody(heavy)
  return w
}

// 11 -------------------------------------------------------- rube goldberg

function rubeGoldberg(): World {
  const w = new World()
  ground(w, 0, -1.5, 12)
  // a few starter pieces to play with
  const ramp = new Body(boxShape(3, 0.15), { ...woodMat, friction: 0.15 })
  ramp.setKind('static')
  ramp.pos.set(-4, 1.2)
  ramp.angle = -0.4
  w.addBody(ramp)
  const ball = new Body(circleShape(0.15), { ...ballMat })
  ball.pos.set(-5.2, 1.9)
  w.addBody(ball)
  const springMass = new Body(boxShape(0.6, 0.6), { ...woodMat, density: 2 })
  springMass.pos.set(2, 1.2)
  w.addBody(springMass)
  w.addConstraint(new SpringConstraint(null, springMass, new Vec2(2, 2.6), new Vec2(0, 0.3), 60, 1, 1.4))
  return w
}

// 12 ----------------------------------------------------------- blank canvas

function blankCanvas(gravity: boolean = true): World {
  const w = new World(gravity ? {} : { gravityMode: 'none' })
  return w
}

export const PRESETS: Preset[] = [
  { id: 'blank', name: 'Blank Canvas', description: 'Empty world, Earth gravity', build: () => blankCanvas(true) },
  { id: 'blank-zero-g', name: 'Blank (zero-g)', description: 'Empty world, no gravity', build: () => blankCanvas(false) },
  { id: 'cradle', name: "Newton's Cradle", description: 'Momentum transfer through elastic collisions', build: newtonsCradle },
  { id: 'double-pendulum', name: 'Double Pendulum', description: 'Two near-identical pendulums diverge into chaos', build: doublePendulum },
  { id: 'projectile', name: 'Projectile Range', description: 'Live parabola with max-height and range readouts', build: projectileRange },
  { id: 'inclined', name: 'Inclined Plane & Friction', description: 'Slip threshold: mu_s vs tan(θ)', build: inclinedPlane },
  { id: 'spring', name: 'Spring-Mass Oscillator', description: 'SHM, live period vs T = 2π√(m/k)', build: springOscillator },
  { id: 'collision', name: 'Collision Lab', description: 'Elastic collision, momentum & KE readouts', build: collisionLab },
  { id: 'orbital', name: 'Orbital Mechanics', description: 'N-body sun/earth/moon + eccentric comet (AU, yr)', build: orbitalMechanics },
  { id: 'galton', name: 'Galton Board', description: 'Binomial distribution from many small collisions', build: galtonBoard },
  { id: 'cloth', name: 'Cloth Drape', description: 'PBD cloth settling over a sphere, breakable threads', build: clothDrape },
  { id: 'fluid', name: 'Fluid Tank', description: 'SPH water; buoyancy emerges from pressure', build: fluidTank },
  { id: 'rube', name: 'Rube Goldberg', description: 'Starter pieces + the full toolbox', build: rubeGoldberg },
]

export function findPreset(id: string): Preset | undefined {
  return PRESETS.find((p) => p.id === id)
}

export { hullShape, RopeConstraint, WeldConstraint }
