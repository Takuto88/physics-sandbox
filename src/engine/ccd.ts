import { Body } from './body'
import { Vec2 } from './vec'

/**
 * Continuous collision detection for small, fast bodies (bullets, marbles).
 * Swept circle (radius r) vs the other body's shape along the displacement
 * for one substep. Returns the earliest time-of-impact t in [0, 1] and the
 * contact normal (pointing from the other body toward the moving one), or
 * null when no collision occurs.
 */
export function sweepCircle(
  p: Vec2,
  disp: Vec2,
  r: number,
  other: Body,
  outT = { t: 0, nx: 0, ny: 0 },
): { t: number; nx: number; ny: number } | null {
  const s = other.shape
  // container object: closures do not participate in TS control-flow narrowing
  const best: { hit: { t: number; nx: number; ny: number } | null } = { hit: null }

  const consider = (t: number, nx: number, ny: number) => {
    if (t <= 0 || t > 1) return
    if (!best.hit || t < best.hit.t) best.hit = { t, nx, ny }
  }

  if (s.type === 'circle') {
    // |p + d t - c| = r + R
    const R = s.radius + r
    const ox = p.x - other.pos.x
    const oy = p.y - other.pos.y
    const a = disp.x * disp.x + disp.y * disp.y
    const b = 2 * (ox * disp.x + oy * disp.y)
    const c = ox * ox + oy * oy - R * R
    if (c > 0) {
      const disc = b * b - 4 * a * c
      if (disc < 0) return null
      const t = (-b - Math.sqrt(disc)) / (2 * a)
      if (t > 0 && t <= 1) {
        const nx = (p.x + disp.x * t - other.pos.x) / (R)
        const ny = (p.y + disp.y * t - other.pos.y) / (R)
        best.hit = { t, nx, ny }
      }
    }
    return best.hit
  }

  // Polygon: capsule sweep per edge (line part + endpoint circles).
  const cos = Math.cos(-other.angle)
  const sin = Math.sin(-other.angle)
  // Transform motion into polygon local space.
  const lp = { x: p.x - other.pos.x, y: p.y - other.pos.y }
  const lpx = lp.x * cos - lp.y * sin
  const lpy = lp.x * sin + lp.y * cos
  const ldx = disp.x * cos - disp.y * sin
  const ldy = disp.x * sin + disp.y * cos

  const verts = s.verts
  const n = verts.length
  for (let i = 0; i < n; i++) {
    const A = verts[i]
    const B = verts[(i + 1) % n]
    const ex = B.x - A.x
    const ey = B.y - A.y
    const elen = Math.hypot(ex, ey)
    const nxl = ey / elen // outward normal (CCW)
    const nyl = -ex / elen

    // Line part: signed distance from the moving center to the infinite line.
    let s0 = (lpx - A.x) * nxl + (lpy - A.y) * nyl
    let nxL = nxl
    let nyL = nyl
    if (s0 < 0) {
      s0 = -s0
      nxL = -nxL
      nyL = -nyL
    }
    if (s0 > r) {
      // Approaching from the far side: solve s(t) = r with s decreasing.
      const sDot = ldx * nxL + ldy * nyL
      if (sDot < 0) {
        const t = (s0 - r) / -sDot
        if (t > 0 && t <= 1) {
          // Check the closest point is on the segment at time t.
          const px = lpx + ldx * t
          const py = lpy + ldy * t
          let u = (px - A.x) * (ex / elen) + (py - A.y) * (ey / elen)
          if (u >= 0 && u <= elen) {
            // Rotate normal back to world space.
            const wx = Math.cos(other.angle) * nxL - Math.sin(other.angle) * nyL
            const wy = Math.sin(other.angle) * nxL + Math.cos(other.angle) * nyL
            consider(t, wx, wy)
          }
        }
      }
    } else {
      // Already within the slab: check the line hit with s(t) = -r (back side)
      // is irrelevant; endpoints cover the entry.
    }

    // Endpoint circles.
    for (const E of [A, B]) {
      const ox = lpx - E.x
      const oy = lpy - E.y
      const a = ldx * ldx + ldy * ldy
      const b = 2 * (ox * ldx + oy * ldy)
      const c = ox * ox + oy * oy - r * r
      if (c > 0 && a > 1e-12) {
        const disc = b * b - 4 * a * c
        if (disc < 0) continue
        const t = (-b - Math.sqrt(disc)) / (2 * a)
        if (t > 0 && t <= 1) {
          // Contact normal: from endpoint toward the circle center at hit time,
          // rotated from local to world space.
          const ldx2 = (ox + ldx * t) / r
          const ldy2 = (oy + ldy * t) / r
          consider(
            t,
            Math.cos(other.angle) * ldx2 - Math.sin(other.angle) * ldy2,
            Math.sin(other.angle) * ldx2 + Math.cos(other.angle) * ldy2,
          )
        }
      }
    }
  }
  if (best.hit) {
    outT.t = best.hit.t
    outT.nx = best.hit.nx
    outT.ny = best.hit.ny
    return best.hit
  }
  return null
}
