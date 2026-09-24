import { Body } from './body'

/**
 * Shared point-vs-body collision: returns the penetration correction that
 * pushes a circle of radius r out of the body, plus the contact normal
 * (pointing out of the body surface toward the point). Null when clear.
 * Used by cloth, blobs, and sand.
 */
export function collidePointWithBody(
  p: { x: number; y: number },
  r: number,
  b: Body,
): { x: number; y: number; nx: number; ny: number } | null {
  const s = b.shape
  if (s.type === 'circle') {
    const dx = p.x - b.pos.x
    const dy = p.y - b.pos.y
    const d2 = dx * dx + dy * dy
    const rr = r + s.radius
    if (d2 >= rr * rr || d2 < 1e-12) return null
    const d = Math.sqrt(d2)
    return { x: ((rr - d) / d) * dx, y: ((rr - d) / d) * dy, nx: dx / d, ny: dy / d }
  }
  const cos = Math.cos(-b.angle)
  const sin = Math.sin(-b.angle)
  const dx = p.x - b.pos.x
  const dy = p.y - b.pos.y
  const lx = dx * cos - dy * sin
  const ly = dx * sin + dy * cos
  // Closest point on the polygon (local space).
  let bestD2 = Infinity
  let bx = 0
  let by = 0
  for (let i = 0; i < s.verts.length; i++) {
    const a = s.verts[i]
    const c = s.verts[(i + 1) % s.verts.length]
    const ex = c.x - a.x
    const ey = c.y - a.y
    let t = ((lx - a.x) * ex + (ly - a.y) * ey) / (ex * ex + ey * ey)
    t = Math.max(0, Math.min(1, t))
    const px = a.x + ex * t
    const py = a.y + ey * t
    const d2 = (lx - px) ** 2 + (ly - py) ** 2
    if (d2 < bestD2) {
      bestD2 = d2
      bx = px
      by = py
    }
  }
  const inside = pointInPolyLocal(lx, ly, s.verts)
  const cosW = Math.cos(b.angle)
  const sinW = Math.sin(b.angle)
  const ldx = lx - bx
  const ldy = ly - by
  const dl = Math.hypot(ldx, ldy) || 1e-9
  // (nxl, nyl) points from the closest surface point toward the particle:
  // outward for a point outside, but INWARD for a point inside the polygon.
  // An interior point must be pushed OUT (toward the surface), so flip it —
  // otherwise a particle that slipped into a wall is driven deeper and lost.
  let nxl = ldx / dl
  let nyl = ldy / dl
  if (inside) {
    nxl = -nxl
    nyl = -nyl
    const pen = r + dl
    const wx = cosW * nxl - sinW * nyl
    const wy = sinW * nxl + cosW * nyl
    return { x: wx * pen, y: wy * pen, nx: wx, ny: wy }
  }
  if (bestD2 >= r * r) return null
  const pen = r - Math.sqrt(bestD2)
  const wx = cosW * nxl - sinW * nyl
  const wy = sinW * nxl + cosW * nyl
  return { x: wx * pen, y: wy * pen, nx: wx, ny: wy }
}

export function pointInPolyLocal(x: number, y: number, verts: { x: number; y: number }[]): boolean {
  for (let i = 0; i < verts.length; i++) {
    const a = verts[i]
    const c = verts[(i + 1) % verts.length]
    if ((c.x - a.x) * (y - a.y) - (c.y - a.y) * (x - a.x) < 0) return false
  }
  return true
}
