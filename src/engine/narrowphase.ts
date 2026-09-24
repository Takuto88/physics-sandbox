import { Vec2 } from './vec'
import { Body } from './body'
import { PolygonShape, CircleShape } from './shapes'
import { Manifold, makeContact } from './contact'
import { combineRestitution, combineFriction } from './material'

/**
 * Narrowphase. Produces manifolds with up to 2 contact points each.
 *
 *  - circle/circle: analytic
 *  - circle/polygon: closest-feature test with inside handling
 *  - polygon/polygon: SAT reference/incident face with Sutherland-Hodgman
 *    clipping (2 contact points)
 *  - freeform convex hulls: GJK distance test + EPA penetration (used when
 *    both bodies carry hull shapes; SAT remains the reference implementation)
 */

export function generateManifold(a: Body, b: Body): Manifold | null {
  const sa = a.shape
  const sb = b.shape
  const m: Manifold = { a, b, points: [] }
  const rest = combineRestitution(a.material, b.material)
  const fric = combineFriction(a.material, b.material)

  if (sa.type === 'circle' && sb.type === 'circle') {
    circleCircle(a, b, m, rest, fric)
  } else if (sa.type === 'circle') {
    // computed n points poly->circle = B->A, so negate to get A->B
    circlePolygon(a, b, m, rest, fric, -1)
  } else if (sb.type === 'circle') {
    // computed n points poly->circle = A->B, keep
    circlePolygon(b, a, m, rest, fric, 1)
  } else {
    if (sa.hull && sb.hull) {
      gjkEpa(a, b, m, rest, fric)
    } else {
      polygonPolygon(a, b, m, rest, fric)
    }
  }
  return m.points.length > 0 ? m : null
}

// ---------------------------------------------------------------- circles

function circleCircle(a: Body, b: Body, m: Manifold, rest: number, fric: number): void {
  const sa = a.shape as CircleShape
  const sb = b.shape as CircleShape
  const ca = a.pos
  const cb = b.pos
  const dx = cb.x - ca.x
  const dy = cb.y - ca.y
  const d2 = dx * dx + dy * dy
  const r = sa.radius + sb.radius
  if (d2 > r * r) return
  const d = Math.sqrt(d2)
  const n = d > 1e-9 ? new Vec2(dx / d, dy / d) : new Vec2(1, 0)
  const cpos = new Vec2(ca.x + n.x * sa.radius, ca.y + n.y * sa.radius)
  m.points.push(makeContact(cpos, n, r - d, rest, fric, 0))
}

// ------------------------------------------------------------- circle/poly

const tmpVerts: Vec2[] = []

function polygonWorldVerts(poly: Body, out: Vec2[]): Vec2[] {
  out.length = 0
  const s = poly.shape as PolygonShape
  const cos = Math.cos(poly.angle)
  const sin = Math.sin(poly.angle)
  for (const v of s.verts) {
    out.push(new Vec2(poly.pos.x + v.x * cos - v.y * sin, poly.pos.y + v.x * sin + v.y * cos))
  }
  return out
}

function polygonWorldNormals(poly: Body, out: Vec2[]): Vec2[] {
  out.length = 0
  const s = poly.shape as PolygonShape
  const cos = Math.cos(poly.angle)
  const sin = Math.sin(poly.angle)
  for (const n of s.normals) {
    out.push(new Vec2(n.x * cos - n.y * sin, n.x * sin + n.y * cos))
  }
  return out
}

/**
 * Circle `a` vs polygon `b`. `flip` = +1 keeps normal A->B, -1 reverses it
 * (used when the arguments were swapped so the polygon is first).
 */
function circlePolygon(circle: Body, poly: Body, m: Manifold, rest: number, fric: number, flip: number): void {
  const verts = polygonWorldVerts(poly, tmpVerts)
  const normals = polygonWorldNormals(poly, [])
  const c = circle.pos
  const r = (circle.shape as { radius: number }).radius

  // Edge with maximum separation from the circle center.
  let bestSep = -Infinity
  let bestI = 0
  for (let i = 0; i < verts.length; i++) {
    const n = normals[i]
    const v = verts[i]
    const sep = n.x * (c.x - v.x) + n.y * (c.y - v.y)
    if (sep > bestSep) {
      bestSep = sep
      bestI = i
    }
  }

  let n: Vec2
  let cpos: Vec2
  let pen: number

  if (bestSep < 0) {
    // Center inside polygon: push out along the support face.
    n = normals[bestI]
    cpos = new Vec2(c.x - n.x * r, c.y - n.y * r)
    pen = -bestSep + r
  } else {
    const v1 = verts[bestI]
    const v2 = verts[(bestI + 1) % verts.length]
    // Corner case: the center lies outside the side plane of an adjacent
    // edge, i.e. beyond the vertex (dot with that edge's outward normal > 0).
    const prev = (bestI - 1 + verts.length) % verts.length
    const nPrev = normals[prev]
    const nNext = normals[(bestI + 1) % verts.length]
    const nearPrev = nPrev.x * (c.x - v1.x) + nPrev.y * (c.y - v1.y) > 0
    const nearNext = nNext.x * (c.x - v2.x) + nNext.y * (c.y - v2.y) > 0

    if (nearPrev || nearNext) {
      // Closest vertex test on both boundary vertices of the face.
      let bx = v1.x
      let by = v1.y
      let bd = (c.x - v1.x) ** 2 + (c.y - v1.y) ** 2
      const d2 = (c.x - v2.x) ** 2 + (c.y - v2.y) ** 2
      if (d2 < bd) {
        bx = v2.x
        by = v2.y
        bd = d2
      }
      const d = Math.sqrt(bd)
      if (d > r) return
      cpos = new Vec2(bx, by)
      n = d > 1e-9 ? new Vec2((c.x - bx) / d, (c.y - by) / d) : normals[bestI].clone()
      pen = r - d
    } else {
      // Face contact: project center onto the edge segment.
      const ex = v2.x - v1.x
      const ey = v2.y - v1.y
      const len2 = ex * ex + ey * ey
      let t = ((c.x - v1.x) * ex + (c.y - v1.y) * ey) / len2
      t = Math.max(0, Math.min(1, t))
      const px = v1.x + ex * t
      const py = v1.y + ey * t
      const ddx = c.x - px
      const ddy = c.y - py
      const d = Math.hypot(ddx, ddy)
      if (d > r) return
      cpos = new Vec2(px, py)
      n = d > 1e-9 ? new Vec2(ddx / d, ddy / d) : normals[bestI].clone()
      pen = r - d
    }
  }

  if (flip < 0) n = n.negate()
  m.points.push(makeContact(cpos, n, pen, rest, fric, bestI * 31 + (flip < 0 ? 1 : 0)))
}

// ------------------------------------------------------------ poly/poly SAT

function polygonPolygon(a: Body, b: Body, m: Manifold, rest: number, fric: number): void {
  const va = polygonWorldVerts(a, [])
  const vb = polygonWorldVerts(b, [])
  const na = polygonWorldNormals(a, [])
  const nb = polygonWorldNormals(b, [])

  // SAT. For a reference axis n (outward normal of A's edge i): A's
  // projection extends to n . v_i (the edge itself); B must extend below it,
  // i.e. min_B(n . v) < n . v_i. sep = the positive overlap depth.
  // The reference axis is the one with the LEAST overlap (min sep).
  let sepA = Infinity
  let axisA = 0
  for (let i = 0; i < na.length; i++) {
    const n = na[i]
    let bmin = Infinity
    let bvx = 0
    let bvy = 0
    for (let j = 0; j < vb.length; j++) {
      const d = n.x * vb[j].x + n.y * vb[j].y
      if (d < bmin) {
        bmin = d
        bvx = vb[j].x
        bvy = vb[j].y
      }
    }
    const v = va[i]
    const sep = n.x * (v.x - bvx) + n.y * (v.y - bvy)
    if (sep < 0) return
    if (sep < sepA) {
      sepA = sep
      axisA = i
    }
  }

  let sepB = Infinity
  let axisB = 0
  for (let i = 0; i < nb.length; i++) {
    const n = nb[i]
    let amin = Infinity
    let avx = 0
    let avy = 0
    for (let j = 0; j < va.length; j++) {
      const d = n.x * va[j].x + n.y * va[j].y
      if (d < amin) {
        amin = d
        avx = va[j].x
        avy = va[j].y
      }
    }
    const v = vb[i]
    const sep = n.x * (v.x - avx) + n.y * (v.y - avy)
    if (sep < 0) return
    if (sep < sepB) {
      sepB = sep
      axisB = i
    }
  }

  // Reference face = axis with the least overlap (deepest contact), small
  // bias toward A for stability.
  const refA = sepA <= sepB + 1e-4
  const refVerts = refA ? va : vb
  const refNormals = refA ? na : nb
  const incVerts = refA ? vb : va
  const refN = refNormals[refA ? axisA : axisB]
  const refFace = refA ? axisA : axisB
  const refV1 = refVerts[refFace]
  const refV2 = refVerts[(refFace + 1) % refVerts.length]

  // Incident face: most anti-parallel edge on the other polygon.
  const incNormals = refA ? nb : na
  let bestDot = Infinity
  let incI = 0
  for (let i = 0; i < incNormals.length; i++) {
    const d = incNormals[i].x * refN.x + incNormals[i].y * refN.y
    if (d < bestDot) {
      bestDot = d
      incI = i
    }
  }
  let i1 = incVerts[incI]
  let i2 = incVerts[(incI + 1) % incVerts.length]

  // Clip incident segment against the two side planes of the reference face.
  const sidePlaneNormal = new Vec2(refV2.x - refV1.x, refV2.y - refV1.y).normalize()
  let clipped = clipSegment(i1.x, i1.y, i2.x, i2.y, sidePlaneNormal.x, sidePlaneNormal.y, -(sidePlaneNormal.x * refV1.x + sidePlaneNormal.y * refV1.y))
  if (clipped.length < 2) return
  const flipped = clipSegment(clipped[0], clipped[1], clipped[2], clipped[3], -sidePlaneNormal.x, -sidePlaneNormal.y, (sidePlaneNormal.x * refV2.x + sidePlaneNormal.y * refV2.y))
  if (flipped.length < 2) return

  // Keep points behind the reference face plane.
  const refOffset = refN.x * refV1.x + refN.y * refV1.y
  for (let i = 0; i < 2; i++) {
    const px = flipped[i * 2]
    const py = flipped[i * 2 + 1]
    const sep = refN.x * px + refN.y * py - refOffset
    if (sep <= 1e-9) {
      const normal = refA ? refN.clone() : refN.clone().negate() // always A -> B
      m.points.push(makeContact(new Vec2(px, py), normal, -sep, rest, fric, (refA ? axisA : 1000 + axisB) * 13 + i))
    }
  }
  if (m.points.length === 0) return
}

/** Sutherland-Hodgman clip of segment (p1,p2) against half-space n.p <= offset. Returns up to 4 numbers. */
function clipSegment(
  p1x: number,
  p1y: number,
  p2x: number,
  p2y: number,
  nx: number,
  ny: number,
  offset: number,
): number[] {
  const out: number[] = []
  const d1 = nx * p1x + ny * p1y - offset
  const d2 = nx * p2x + ny * p2y - offset
  if (d1 <= 0) out.push(p1x, p1y)
  if (d2 <= 0) out.push(p2x, p2y)
  if (d1 * d2 < 0) {
    const t = d1 / (d1 - d2)
    out.push(p1x + t * (p2x - p1x), p1y + t * (p2y - p1y))
  }
  return out
}

// --------------------------------------------------------------- GJK + EPA

interface SupportResult {
  d: Vec2 // difference-space point (sa - sb)
  sa: Vec2
  sb: Vec2
}

function supportMap(a: Body, b: Body, dir: Vec2): SupportResult {
  // a: maximize dot(dir, p); b: minimize dot(dir, p)
  let sa: Vec2
  if (a.shape.type === 'circle') {
    sa = new Vec2(a.pos.x + dir.x * a.shape.radius, a.pos.y + dir.y * a.shape.radius)
  } else {
    let best = -Infinity
    sa = a.localToWorld(a.shape.verts[0])
    for (const v of a.shape.verts) {
      const w = a.localToWorld(v)
      const d = dir.x * w.x + dir.y * w.y
      if (d > best) {
        best = d
        sa = w
      }
    }
  }
  let sb: Vec2
  if (b.shape.type === 'circle') {
    sb = new Vec2(b.pos.x - dir.x * b.shape.radius, b.pos.y - dir.y * b.shape.radius)
  } else {
    let best = Infinity
    sb = b.localToWorld(b.shape.verts[0])
    for (const v of b.shape.verts) {
      const w = b.localToWorld(v)
      const d = dir.x * w.x + dir.y * w.y
      if (d < best) {
        best = d
        sb = w
      }
    }
  }
  return { d: new Vec2(sa.x - sb.x, sa.y - sb.y), sa, sb }
}

type GjkResult =
  | { overlap: false; dist: number; pa: Vec2; pb: Vec2 }
  | { overlap: true; simplex: SupportResult[] }

/**
 * GJK in 2D via closest-point-on-simplex. Returns either the separation
 * (distance + closest points) or, when the shapes overlap, a simplex that
 * contains the origin (used to seed EPA).
 */
export function gjk(a: Body, b: Body): GjkResult {
  let dir = new Vec2(b.pos.x - a.pos.x, b.pos.y - a.pos.y)
  if (dir.lenSq() < 1e-12) dir = new Vec2(1, 0)
  dir.normalize()

  let simplex: SupportResult[] = [supportMap(a, b, dir)]
  for (let iter = 0; iter < 32; iter++) {
    const res = closestOnSimplex(simplex)
    if (res.distSq < 1e-14) {
      return { overlap: true, simplex: res.simplex }
    }
    const closest = res.closest
    const s = supportMap(a, b, new Vec2(-closest.x, -closest.y))
    const advance = (s.d.x - closest.x) * -closest.x + (s.d.y - closest.y) * -closest.y
    if (advance < 1e-10) {
      return { overlap: false, dist: Math.sqrt(res.distSq), pa: s.sa, pb: s.sb }
    }
    simplex = [s, ...res.simplex]
    if (simplex.length > 3) simplex.length = 3
  }
  // Should not happen; report as non-overlapping with a conservative distance.
  const c = closestOnSimplex(simplex).closest
  return { overlap: false, dist: c.len(), pa: simplex[0].sa, pb: simplex[0].sb }
}

function closestOnSimplex(pts: SupportResult[]): { distSq: number; closest: Vec2; simplex: SupportResult[] } {
  const p = pts.map((q) => q.d)
  if (p.length === 1) {
    return { distSq: p[0].lenSq(), closest: p[0].clone(), simplex: [pts[0]] }
  }
  if (p.length === 2) {
    return closestOnSegment(p[0], p[1], [pts[0], pts[1]])
  }
  // Triangle: determine which edge the origin is outside of.
  const a = p[0]
  const b = p[1]
  const c = p[2]
  const area2 = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)
  if (Math.abs(area2) < 1e-14) {
    return closestOnSegment(a, c, [pts[0], pts[2]])
  }
  const s = Math.sign(area2)
  // Edge a->b: origin outside (toward b side) if cross(b-a, o-a) has the
  // opposite sign to the triangle winding.
  const abx = b.x - a.x
  const aby = b.y - a.y
  if ((abx * -a.y - aby * -a.x) * s < 0) {
    return closestOnSegment(a, b, [pts[0], pts[1]])
  }
  const bcx = c.x - b.x
  const bcy = c.y - b.y
  if ((bcx * -b.y - bcy * -b.x) * s < 0) {
    return closestOnSegment(b, c, [pts[1], pts[2]])
  }
  // Origin inside the triangle.
  return { distSq: 0, closest: new Vec2(), simplex: [pts[0], pts[1], pts[2]] }
}

function closestOnSegment(p: Vec2, q: Vec2, src: SupportResult[]): { distSq: number; closest: Vec2; simplex: SupportResult[] } {
  const dx = q.x - p.x
  const dy = q.y - p.y
  const lenSq = dx * dx + dy * dy
  if (lenSq < 1e-14) return { distSq: p.lenSq(), closest: p.clone(), simplex: [src[0]] }
  let t = (-p.x * dx - p.y * dy) / lenSq
  if (t <= 0) return { distSq: p.lenSq(), closest: p.clone(), simplex: [src[0]] }
  if (t >= 1) return { distSq: q.lenSq(), closest: q.clone(), simplex: [src[1]] }
  const cl = new Vec2(p.x + t * dx, p.y + t * dy)
  return { distSq: cl.lenSq(), closest: cl, simplex: [src[0], src[1]] }
}

/**
 * EPA on a GJK simplex known to contain the origin. Returns penetration
 * depth and the A->B normal plus closest feature points.
 */
export function epaPenetration(a: Body, b: Body, simplex: SupportResult[]): { depth: number; normal: Vec2; pa: Vec2; pb: Vec2 } {
  let poly = simplex.map((s) => ({ d: s.d.clone(), sa: s.sa.clone(), sb: s.sb.clone() }))
  for (let iter = 0; iter < 32; iter++) {
    // Edge with minimum distance to origin.
    let minDist = Infinity
    let minEdge = 0
    const n = poly.length
    for (let i = 0; i < n; i++) {
      const p = poly[i]
      const q = poly[(i + 1) % n]
      const ex = q.d.x - p.d.x
      const ey = q.d.y - p.d.y
      const elen = Math.hypot(ex, ey)
      if (elen < 1e-12) continue
      // outward normal (polygon in difference space winds CCW around origin after GJK)
      let nx = ey / elen
      let ny = -ex / elen
      // outward normal: must point away from the origin (n . p > 0)
      if (nx * p.d.x + ny * p.d.y < 0) {
        nx = -nx
        ny = -ny
      }
      const dist = nx * p.d.x + ny * p.d.y
      if (dist < minDist) {
        minDist = dist
        minEdge = i
      }
    }
    const p = poly[minEdge]
    const q = poly[(minEdge + 1) % poly.length]
    const ex = q.d.x - p.d.x
    const ey = q.d.y - p.d.y
    const elen = Math.hypot(ex, ey)
    let nx = ey / elen
    let ny = -ex / elen
    // outward normal: must point away from the origin (n . p > 0)
    if (nx * p.d.x + ny * p.d.y < 0) {
      nx = -nx
      ny = -ny
    }
    const dir = new Vec2(nx, ny)
    const s = supportMap(a, b, dir)
    // Converged when the new support point does not advance past the edge.
    const advance = (s.d.x - p.d.x) * nx + (s.d.y - p.d.y) * ny
    if (advance < 1e-9) {
      return { depth: minDist, normal: dir, pa: p.sa, pb: p.sb }
    }
    // Split edge at minEdge, inserting the new vertex.
    poly.splice(minEdge + 1, 0, { d: s.d.clone(), sa: s.sa.clone(), sb: s.sb.clone() })
  }
  return { depth: 0, normal: new Vec2(1, 0), pa: poly[0].sa, pb: poly[0].sb }
}

function gjkEpa(a: Body, b: Body, m: Manifold, rest: number, fric: number): void {
  const res = gjk(a, b)
  if (!res.overlap) return
  const pen = epaPenetration(a, b, res.simplex)
  const cpos = new Vec2((pen.pa.x + pen.pb.x) / 2, (pen.pa.y + pen.pb.y) / 2)
  m.points.push(makeContact(cpos, pen.normal, pen.depth, rest, fric, 7))
}
