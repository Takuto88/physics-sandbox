import { Vec2 } from './vec'
import { Body, resetBodyIds } from './body'
import { Shape, circleShape, boxShape, hullShape, PolygonShape, circleShape as cs } from './shapes'
import { Material } from './material'
import {
  Constraint,
  DistanceConstraint,
  RopeConstraint,
  RevoluteConstraint,
  PrismaticConstraint,
  WeldConstraint,
  SpringConstraint,
} from './constraints'
import { World, WorldConfig, Emitter, DT } from './world'
import { Cloth } from './softbody'
import { ShapeMatchingBlob } from './softbody'
import { Fluid, FluidConfig } from './fluid'
import { RNG } from './rand'

export interface SceneJSON {
  version: 1
  seed: number
  config: Partial<WorldConfig> & { gravityMode?: 'uniform' | 'nbody' | 'none' }
  bodies: Array<{
    kind?: 'dynamic' | 'static' | 'kinematic'
    label?: string
    x: number
    y: number
    angle?: number
    vx?: number
    vy?: number
    w?: number
    material: Partial<Material>
    shape: { type: 'circle'; r: number } | { type: 'polygon'; verts: number[][]; hull?: boolean }
  }>
  constraints: Array<{
    type: 'distance' | 'rope' | 'revolute' | 'prismatic' | 'weld' | 'spring'
    a: number | null
    b: number | null
    la: [number, number]
    lb: [number, number]
    rest?: number
    k?: number
    c?: number
    axis?: [number, number]
    motorOn?: boolean
    motorSpeed?: number
    motorForce?: number
    limit?: [number, number]
    label?: string
  }>
  cloths?: Array<{
    cols: number
    rows: number
    spacing: number
    color?: string
    x?: number
    y?: number
    particles?: number[][]
    pinned?: number[]
  }>
  blobs?: Array<{ radius: number; color?: string; x: number; y: number }>
  fluid?: { points: number[][]; config?: Partial<FluidConfig> }
  emitters?: Array<Record<string, unknown>>
}

// ------------------------------------------------------------- world -> json

export function worldToScene(w: World): SceneJSON {
  const bodies = w.bodies.map((b) => {
    const shape: SceneJSON['bodies'][number]['shape'] =
      b.shape.type === 'circle'
        ? { type: 'circle', r: b.shape.radius }
        : {
            type: 'polygon',
            verts: (b.shape as PolygonShape).verts.map((v) => [v.x, v.y]),
            hull: (b.shape as PolygonShape).hull,
          }
    return {
      kind: b.kind,
      label: b.label || undefined,
      x: b.pos.x,
      y: b.pos.y,
      angle: b.angle,
      vx: b.vel.x,
      vy: b.vel.y,
      w: b.angVel,
      material: { ...b.material, density: b.material.density },
      shape,
      fixedRotation: b.fixedRotation || undefined,
      linearDamping: b.linearDamping || undefined,
      angularDamping: b.angularDamping || undefined,
    } as SceneJSON['bodies'][number]
  })
  const constraints = w.constraints.map((c) => {
    const base = {
      a: c.a ? c.a.id : null,
      b: c.b ? c.b.id : null,
      la: [c.localA.x, c.localA.y] as [number, number],
      lb: [c.localB.x, c.localB.y] as [number, number],
      label: c.label || undefined,
    }
    if (c instanceof SpringConstraint)
      return { ...base, type: 'spring', k: c.stiffness, c: c.damping, rest: c.rest } as SceneJSON['constraints'][number]
    if (c instanceof PrismaticConstraint)
      return { ...base, type: 'prismatic', axis: [c.axis.x, c.axis.y] } as SceneJSON['constraints'][number]
    if (c instanceof RevoluteConstraint)
      return {
        ...base,
        type: 'revolute',
        motorOn: c.motorOn,
        motorSpeed: c.motorSpeed,
        motorForce: c.motorForce,
        limit: c.angleLimit ? c.angleLimit : undefined,
      } as SceneJSON['constraints'][number]
    if (c instanceof DistanceConstraint || c instanceof RopeConstraint)
      return {
        ...base,
        type: c instanceof RopeConstraint ? 'rope' : 'distance',
        rest: (c as DistanceConstraint).rest,
      } as SceneJSON['constraints'][number]
    return { ...base, type: 'weld' } as SceneJSON['constraints'][number]
  })
  return {
    version: 1,
    seed: w.config.seed,
    config: {
      gravityMode: w.config.gravityMode,
      gravityX: w.config.gravityX,
      gravityY: w.config.gravityY,
      G: w.config.G,
      airDrag: w.config.airDrag,
      baseTimeScale: w.config.baseTimeScale,
      substeps: w.config.substeps,
      integrator: w.config.integrator,
      solverIterations: w.config.solverIterations,
    },
    bodies,
    constraints,
    cloths: w.cloths.map((c) => ({
      cols: c.cols,
      rows: c.rows,
      spacing: c.spacing,
      color: c.color,
      particles: c.particles.map((p) => [p.x, p.y]),
      pinned: c.particles.map((p, i) => (p.invMass === 0 ? i : -1)).filter((i) => i >= 0),
    })),
    blobs: w.blobs.map((b) => ({
      radius: b.radius,
      color: b.color,
      x: b.particles[0]?.x ?? 0,
      y: b.particles[0]?.y ?? 0,
    })),
    fluid: w.fluid
      ? {
          points: w.fluid.points.map((p) => [p.x, p.y, p.vx, p.vy]),
          config: { ...w.fluid.config },
        }
      : undefined,
    emitters: w.emitters.length
      ? w.emitters.map((e) => ({
          x: e.x,
          y: e.y,
          shape:
            e.shape.type === 'circle'
              ? { type: 'circle', r: e.shape.radius }
              : { type: 'polygon', verts: (e.shape as PolygonShape).verts.map((v) => [v.x, v.y]) },
          material: e.material,
          rate: e.rate,
          total: e.total,
          velBase: [e.velBase.x, e.velBase.y],
          velSpread: e.velSpread,
          jitter: e.jitter,
          emitted: e.emitted,
        }))
      : undefined,
  }
}

// --------------------------------------------------------------- json -> world

export function sceneToWorld(scene: SceneJSON): World {
  resetBodyIds()
  const w = new World(scene.config as WorldConfig)
  const byId = new Map<number, Body>()
  scene.bodies.forEach((jb, i) => {
    let shape: Shape
    if (jb.shape.type === 'circle') {
      shape = circleShape(jb.shape.r)
    } else {
      const verts = jb.shape.verts.map((v) => new Vec2(v[0], v[1]))
      shape = jb.shape.hull ? hullShape(verts) : polygonFromLocalVerts(verts)
    }
    const mat: Material = {
      restitution: jb.material.restitution ?? 0.35,
      friction: jb.material.friction ?? 0.4,
      density: jb.material.density ?? 1,
      color: jb.material.color ?? '#7aa2ff',
      glow: jb.material.glow ?? false,
    }
    const b = new Body(shape, mat)
    b.id = i + 1
    b.kind = jb.kind ?? 'dynamic'
    b.recomputeMass()
    b.pos.set(jb.x, jb.y)
    b.angle = jb.angle ?? 0
    b.vel.set(jb.vx ?? 0, jb.vy ?? 0)
    b.angVel = jb.w ?? 0
    b.label = jb.label ?? ''
    b.fixedRotation = (jb as { fixedRotation?: boolean }).fixedRotation ?? false
    b.recomputeMass()
    b.linearDamping = (jb as { linearDamping?: number }).linearDamping ?? 0
    b.angularDamping = (jb as { angularDamping?: number }).angularDamping ?? 0
    b.prevPos.copy(b.pos)
    b.prevAngle = b.angle
    w.addBody(b)
    byId.set(i + 1, b)
  })

  for (const jc of scene.constraints) {
    const a = jc.a !== null ? (byId.get(jc.a) ?? null) : null
    const b = jc.b !== null ? (byId.get(jc.b) ?? null) : null
    const la = new Vec2(jc.la[0], jc.la[1])
    const lb = new Vec2(jc.lb[0], jc.lb[1])
    let c: Constraint
    switch (jc.type) {
      case 'spring':
        c = new SpringConstraint(a, b, la, lb, jc.k ?? 200, jc.c ?? 1, jc.rest)
        break
      case 'prismatic':
        c = new PrismaticConstraint(a, b, la, lb, new Vec2(jc.axis?.[0] ?? 0, jc.axis?.[1] ?? 1))
        break
      case 'revolute': {
        const rc = new RevoluteConstraint(a, b, la, lb)
        rc.motorOn = jc.motorOn ?? false
        rc.motorSpeed = jc.motorSpeed ?? 0
        rc.motorForce = jc.motorForce ?? 100
        rc.angleLimit = jc.limit ?? null
        c = rc
        break
      }
      case 'rope':
        c = new RopeConstraint(a, b, la, lb, jc.rest)
        break
      case 'distance':
        c = new DistanceConstraint(a, b, la, lb, jc.rest)
        break
      default:
        c = new WeldConstraint(a, b, la, lb)
    }
    if (jc.label) c.label = jc.label
    w.addConstraint(c)
  }

  for (const jc of scene.cloths ?? []) {
    const c = new Cloth(jc.cols, jc.rows, jc.spacing, jc.color)
    if (jc.particles) {
      jc.particles.forEach((p, i) => {
        if (i < c.particles.length) {
          c.particles[i].x = c.particles[i].px = p[0]
          c.particles[i].y = c.particles[i].py = p[1]
        }
      })
      for (const pin of jc.pinned ?? []) {
        if (pin < c.particles.length) c.particles[pin].invMass = 0
      }
    } else {
      c.translate(jc.x ?? 0, jc.y ?? 0)
    }
    w.addCloth(c)
  }

  for (const jb of scene.blobs ?? []) {
    const b = new ShapeMatchingBlob(jb.radius, 40, jb.color)
    b.translate(jb.x, jb.y)
    w.addBlob(b)
  }

  if (scene.fluid) {
    const f = new Fluid(scene.fluid.config)
    for (const p of scene.fluid.points) {
      f.points.push({
        x: p[0],
        y: p[1],
        px: p[0],
        py: p[1],
        vx: p[2] ?? 0,
        vy: p[3] ?? 0,
        density: 0,
        nearDensity: 0,
        pressure: 0,
        nearPressure: 0,
      })
    }
    w.setFluid(f)
  }

  for (const je of scene.emitters ?? []) {
    const e = je as unknown as {
      x: number
      y: number
      shape: SceneJSON['bodies'][number]['shape']
      material: Material
      rate: number
      total: number
      velBase: [number, number]
      velSpread: number
      jitter: number
      emitted: number
    }
    const shape: Shape =
      e.shape.type === 'circle'
        ? circleShape(e.shape.r)
        : polygonFromLocalVerts(e.shape.verts.map((v) => new Vec2(v[0], v[1])))
    w.addEmitter({
      x: e.x,
      y: e.y,
      shape,
      material: e.material,
      rate: e.rate,
      total: e.total,
      velBase: new Vec2(e.velBase[0], e.velBase[1]),
      velSpread: e.velSpread,
      jitter: e.jitter,
      elapsed: 0,
      acc: 0,
      emitted: e.emitted,
      active: true,
    })
  }

  w.rng = new RNG(w.config.seed)
  return w
}

/**
 * Rebuild a polygon shape from centroid-local verts without re-hulling
 * (scene round-trip: verts are already the canonical hull).
 */
export function polygonFromLocalVerts(verts: Vec2[]): PolygonShape {
  const n = verts.length
  let signedArea = 0
  for (let i = 0; i < n; i++) {
    const a = verts[i]
    const b = verts[(i + 1) % n]
    signedArea += a.x * b.y - b.x * a.y
  }
  if (signedArea < 0) verts.reverse()
  const normals: Vec2[] = []
  let boundingRadius = 0
  for (let i = 0; i < n; i++) {
    const a = verts[i]
    const b = verts[(i + 1) % n]
    const ex = b.x - a.x
    const ey = b.y - a.y
    const len = Math.hypot(ex, ey)
    normals.push(new Vec2(ey / len, -ex / len))
    boundingRadius = Math.max(boundingRadius, a.len())
  }
  return { type: 'polygon', verts, normals, area: Math.abs(signedArea) / 2, boundingRadius, hull: false }
}

// Convenience builders used by tools and presets.
export function makeCircle(r: number, x: number, y: number, mat?: Partial<Material>): Body {
  const b = new Body(circleShape(r), mat)
  b.pos.set(x, y)
  return b
}

export function makeBox(w: number, h: number, x: number, y: number, mat?: Partial<Material>, angle = 0): Body {
  const b = new Body(boxShape(w, h), mat)
  b.pos.set(x, y)
  b.angle = angle
  return b
}

export { cs, DT }
export type { Emitter }
