import { Vec2 } from './vec'

export interface MassProperties {
  mass: number
  inertia: number
}

/**
 * Collision shapes. Polygon vertices are stored in body-local coordinates,
 * centered on the centroid, counter-clockwise, so `body.pos` is always the
 * center of mass.
 */
export interface CircleShape {
  type: 'circle'
  radius: number
}

export interface PolygonShape {
  type: 'polygon'
  verts: Vec2[] // CCW, local, centroid at origin
  normals: Vec2[] // per edge (verts[i] -> verts[i+1]), outward
  area: number
  boundingRadius: number
  /** true when built from the freeform pencil (convex hull of drawn points) */
  hull: boolean
}

export type Shape = CircleShape | PolygonShape

export function shapeRadius(s: Shape): number {
  return s.type === 'circle' ? s.radius : s.boundingRadius
}

export function circleShape(radius: number): CircleShape {
  return { type: 'circle', radius }
}

/** Axis-aligned rectangle centered at the local origin. */
export function boxShape(w: number, h: number): PolygonShape {
  const hw = w / 2
  const hh = h / 2
  return polygonFromPoints([
    new Vec2(-hw, -hh),
    new Vec2(hw, -hh),
    new Vec2(hw, hh),
    new Vec2(-hw, hh),
  ])
}

function polygonFromPoints(points: Vec2[]): PolygonShape {
  // Convex hull (monotone chain), then translate to centroid.
  const pts = points.map((p) => p.clone()).sort((a, b) => a.x - b.x || a.y - b.y)
  const cross = (o: Vec2, a: Vec2, b: Vec2) => (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x)
  const lower: Vec2[] = []
  for (const p of pts) {
    while (lower.length >= 2 && cross(lower[lower.length - 2], lower[lower.length - 1], p) <= 0) lower.pop()
    lower.push(p)
  }
  const upper: Vec2[] = []
  for (let i = pts.length - 1; i >= 0; i--) {
    const p = pts[i]
    while (upper.length >= 2 && cross(upper[upper.length - 2], upper[upper.length - 1], p) <= 0) upper.pop()
    upper.push(p)
  }
  lower.pop()
  upper.pop()
  const hull = [...lower, ...upper]
  if (hull.length < 3) hull.push(new Vec2(0.01, 0.01), new Vec2(-0.01, 0.01), new Vec2(0, 0.02))

  // Shoelace area + centroid (cx.x = centroid x accumulator, cx.y = y).
  let area = 0
  const cx = new Vec2()
  for (let i = 0; i < hull.length; i++) {
    const a = hull[i]
    const b = hull[(i + 1) % hull.length]
    const d = a.x * b.y - b.x * a.y
    area += d
    cx.x += (a.x + b.x) * d
    cx.y += (a.y + b.y) * d
  }
  area *= 0.5
  if (Math.abs(area) < 1e-12) area = 1e-12
  // Signed shoelace: for CW polygons both cx and area are negative, so the
  // ratio is already correct without extra sign juggling.
  const c = new Vec2(cx.x / (6 * area), cx.y / (6 * area))
  const verts = hull.map((p) => new Vec2(p.x - c.x, p.y - c.y))

  // Ensure CCW.
  let signedArea = 0
  for (let i = 0; i < verts.length; i++) {
    const a = verts[i]
    const b = verts[(i + 1) % verts.length]
    signedArea += a.x * b.y - b.x * a.y
  }
  if (signedArea < 0) {
    verts.reverse()
  }

  const n = verts.length
  const normals: Vec2[] = []
  let boundingRadius = 0
  for (let i = 0; i < n; i++) {
    const a = verts[i]
    const b = verts[(i + 1) % n]
    const ex = b.x - a.x
    const ey = b.y - a.y
    const len = Math.hypot(ex, ey)
    // Outward normal for CCW polygon is (ey, -ex).
    normals.push(new Vec2(ey / len, -ex / len))
    boundingRadius = Math.max(boundingRadius, a.len())
  }

  return { type: 'polygon', verts, normals, area: Math.abs(signedArea) / 2, boundingRadius, hull: false }
}

/** Convex hull of arbitrary drawn points (freeform pencil). */
export function hullShape(points: Vec2[]): PolygonShape {
  const s = polygonFromPoints(points)
  s.hull = true
  return s
}

/**
 * Mass properties from geometry and density (kg/m^2 in 2D).
 * Circle: I = m r^2 / 2.
 * Polygon: standard triangle-fan formula, integrated about the origin then
 * shifted to the centroid with the parallel axis theorem.
 */
export function massProperties(shape: Shape, density: number): MassProperties {
  if (shape.type === 'circle') {
    const mass = density * Math.PI * shape.radius * shape.radius
    return { mass, inertia: (mass * shape.radius * shape.radius) / 2 }
  }
  const v = shape.verts
  let area = 0
  let I = 0
  const n = v.length
  for (let i = 0; i < n; i++) {
    const a = v[i]
    const b = v[(i + 1) % n]
    const d = a.x * b.y - b.x * a.y
    area += d * 0.5
    I += (d / 12) * (a.x * a.x + a.x * b.x + b.x * b.x + a.y * a.y + a.y * b.y + b.y * b.y)
  }
  const mass = density * Math.abs(area)
  return { mass, inertia: density * I }
}
