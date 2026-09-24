import { World } from '../engine/world'
import { Body } from '../engine/body'
import { Camera } from './camera'

/**
 * Canvas2D renderer. Draws the full simulation state in screen space, mapping
 * every point through the camera. Uses the engine's prev/current interpolation
 * (world.alpha + body.prevPos/prevAngle) so motion is smooth at 60 fps even
 * though the physics runs at 120 Hz.
 */
export interface RenderOptions {
  showGrid: boolean
  showVelocity: boolean
  showLabels: boolean
  selectedId: number | null
}

export const DEFAULT_RENDER_OPTS: RenderOptions = {
  showGrid: true,
  showVelocity: false,
  showLabels: true,
  selectedId: null,
}

/** interpolate a body's pose between the previous and current tick. */
function pose(b: Body, alpha: number): { x: number; y: number; a: number } {
  const x = b.prevPos.x + (b.pos.x - b.prevPos.x) * alpha
  const y = b.prevPos.y + (b.pos.y - b.prevPos.y) * alpha
  let da = b.angle - b.prevAngle
  if (da > Math.PI) da -= 2 * Math.PI
  else if (da < -Math.PI) da += 2 * Math.PI
  const a = b.prevAngle + da * alpha
  return { x, y, a }
}

export function render(
  ctx: CanvasRenderingContext2D,
  w: World,
  cam: Camera,
  opts: RenderOptions,
  dpr: number,
): void {
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0)
  ctx.clearRect(0, 0, cam.w, cam.h)

  // Backdrop.
  ctx.fillStyle = '#0b1020'
  ctx.fillRect(0, 0, cam.w, cam.h)

  if (opts.showGrid) drawGrid(ctx, cam)

  const alpha = w.alpha

  drawConstraints(ctx, w, cam)
  for (const c of w.cloths) drawCloth(ctx, cam, c.particles, c.color)
  for (const b of w.blobs) drawBlob(ctx, cam, b)
  if (w.fluid) drawFluid(ctx, w, cam)
  drawParticles(ctx, w, cam)

  for (const b of w.bodies) drawBody(ctx, b, alpha, cam, opts)
}

function drawGrid(ctx: CanvasRenderingContext2D, cam: Camera): void {
  const worldMinX = cam.toWorldX(0)
  const worldMaxX = cam.toWorldX(cam.w)
  const worldMinY = cam.toWorldY(cam.h)
  const worldMaxY = cam.toWorldY(0)

  // choose a grid spacing that keeps ~40 lines on screen
  const raw = (worldMaxX - worldMinX) / 40
  const mag = Math.pow(10, Math.floor(Math.log10(raw)))
  const norm = raw / mag
  const step = (norm < 1.5 ? 1 : norm < 3.5 ? 2 : norm < 7.5 ? 5 : 10) * mag

  ctx.lineWidth = 1
  ctx.strokeStyle = 'rgba(120,140,200,0.08)'
  ctx.beginPath()
  for (let x = Math.floor(worldMinX / step) * step; x <= worldMaxX; x += step) {
    const sx = cam.toScreenX(x)
    ctx.moveTo(sx, 0)
    ctx.lineTo(sx, cam.h)
  }
  for (let y = Math.floor(worldMinY / step) * step; y <= worldMaxY; y += step) {
    const sy = cam.toScreenY(y)
    ctx.moveTo(0, sy)
    ctx.lineTo(cam.w, sy)
  }
  ctx.stroke()

  // axes
  const ox = cam.toScreenX(0)
  const oy = cam.toScreenY(0)
  ctx.strokeStyle = 'rgba(120,140,200,0.22)'
  ctx.beginPath()
  ctx.moveTo(ox, 0)
  ctx.lineTo(ox, cam.h)
  ctx.moveTo(0, oy)
  ctx.lineTo(cam.w, oy)
  ctx.stroke()
}

function drawBody(
  ctx: CanvasRenderingContext2D,
  b: Body,
  alpha: number,
  cam: Camera,
  opts: RenderOptions,
): void {
  const p = pose(b, alpha)
  const { x: sx, y: sy } = cam.toScreen(p.x, p.y)
  const r = cam.scale
  const color = b.material.color
  const selected = opts.selectedId === b.id

  ctx.save()
  if (b.material.glow) {
    ctx.shadowColor = color
    ctx.shadowBlur = 14
  }

  if (b.shape.type === 'circle') {
    const rad = b.shape.radius * r
    ctx.beginPath()
    ctx.arc(sx, sy, rad, 0, Math.PI * 2)
    ctx.fillStyle = color
    ctx.fill()
    ctx.lineWidth = Math.max(1, 0.012 * r)
    ctx.strokeStyle = shade(color, selected ? 1.7 : 1.25)
    ctx.stroke()
  } else {
    const cos = Math.cos(p.a)
    const sin = Math.sin(p.a)
    ctx.beginPath()
    for (let i = 0; i < b.shape.verts.length; i++) {
      const v = b.shape.verts[i]
      const wx = p.x + v.x * cos - v.y * sin
      const wy = p.y + v.x * sin + v.y * cos
      const s = cam.toScreen(wx, wy)
      if (i === 0) ctx.moveTo(s.x, s.y)
      else ctx.lineTo(s.x, s.y)
    }
    ctx.closePath()
    ctx.fillStyle = color
    ctx.fill()
    ctx.lineWidth = Math.max(1, 0.012 * r)
    ctx.strokeStyle = shade(color, selected ? 1.7 : 1.25)
    ctx.stroke()
    // orientation tick so rotation is visible
    const tv = b.shape.verts[0]
    const tx = p.x + tv.x * cos - tv.y * sin
    const ty = p.y + tv.x * sin + tv.y * cos
    const ts = cam.toScreen(tx, ty)
    ctx.beginPath()
    ctx.moveTo(sx, sy)
    ctx.lineTo(ts.x, ts.y)
    ctx.lineWidth = Math.max(1, 0.006 * r)
    ctx.strokeStyle = 'rgba(0,0,0,0.25)'
    ctx.stroke()
  }
  ctx.restore()

  if (selected) {
    ctx.beginPath()
    ctx.arc(sx, sy, b.worldRadius() * r + 6, 0, Math.PI * 2)
    ctx.lineWidth = 1.5
    ctx.setLineDash([4, 3])
    ctx.strokeStyle = '#7dd3fc'
    ctx.stroke()
    ctx.setLineDash([])
  }

  if (opts.showVelocity && b.isDynamic()) {
    const vx = b.vel.x * r * 0.15
    const vy = -b.vel.y * r * 0.15
    ctx.beginPath()
    ctx.moveTo(sx, sy)
    ctx.lineTo(sx + vx, sy + vy)
    ctx.lineWidth = 1.5
    ctx.strokeStyle = 'rgba(125,211,252,0.7)'
    ctx.stroke()
  }

  if (opts.showLabels && b.label) {
    ctx.font = '11px ui-monospace, monospace'
    ctx.fillStyle = 'rgba(220,230,255,0.75)'
    ctx.fillText(b.label, sx + b.worldRadius() * r + 4, sy - 4)
  }
}

function drawConstraints(ctx: CanvasRenderingContext2D, w: World, cam: Camera): void {
  for (const c of w.constraints) {
    const a = c.anchorA()
    const b = c.anchorB()
    const sa = cam.toScreen(a.x, a.y)
    const sb = cam.toScreen(b.x, b.y)
    const isSpring = 'stiffness' in c
    if (isSpring) {
      drawSpring(ctx, sa, sb, (c as unknown as { stretch: number }).stretch, cam.scale)
    } else {
      ctx.beginPath()
      ctx.moveTo(sa.x, sa.y)
      ctx.lineTo(sb.x, sb.y)
      ctx.lineWidth = Math.max(1, 0.012 * cam.scale)
      ctx.strokeStyle = 'rgba(190,200,230,0.7)'
      ctx.stroke()
      // anchor dots
      ctx.fillStyle = 'rgba(190,200,230,0.9)'
      ctx.beginPath()
      ctx.arc(sa.x, sa.y, Math.max(1.5, 0.02 * cam.scale), 0, Math.PI * 2)
      ctx.arc(sb.x, sb.y, Math.max(1.5, 0.02 * cam.scale), 0, Math.PI * 2)
      ctx.fill()
    }
  }
}

function drawSpring(
  ctx: CanvasRenderingContext2D,
  a: { x: number; y: number },
  b: { x: number; y: number },
  stretch: number,
  scale: number,
): void {
  const dx = b.x - a.x
  const dy = b.y - a.y
  const len = Math.hypot(dx, dy) || 1
  const nx = -dy / len
  const ny = dx / len
  const coils = 10
  const amp = Math.min(0.08 * scale, 0.03 * len) * (1 + Math.min(3, Math.abs(stretch) * 4))
  ctx.beginPath()
  ctx.moveTo(a.x, a.y)
  for (let i = 1; i < coils; i++) {
    const t = i / coils
    const off = (i % 2 === 0 ? 1 : -1) * amp
    ctx.lineTo(a.x + dx * t + nx * off, a.y + dy * t + ny * off)
  }
  ctx.lineTo(b.x, b.y)
  ctx.lineWidth = Math.max(1, 0.012 * scale)
  ctx.strokeStyle = 'rgba(250,200,120,0.8)'
  ctx.stroke()
}

function drawCloth(
  ctx: CanvasRenderingContext2D,
  cam: Camera,
  pts: { x: number; y: number }[],
  color: string,
): void {
  ctx.fillStyle = color
  const rad = Math.max(1, 0.012 * cam.scale)
  for (const p of pts) {
    const s = cam.toScreen(p.x, p.y)
    ctx.beginPath()
    ctx.arc(s.x, s.y, rad, 0, Math.PI * 2)
    ctx.fill()
  }
}

function drawBlob(
  ctx: CanvasRenderingContext2D,
  cam: Camera,
  blob: { particles: { x: number; y: number }[]; color: string; radius: number },
): void {
  const rad = Math.max(1, 0.02 * cam.scale)
  ctx.fillStyle = blob.color
  for (const p of blob.particles) {
    const s = cam.toScreen(p.x, p.y)
    ctx.beginPath()
    ctx.arc(s.x, s.y, rad, 0, Math.PI * 2)
    ctx.fill()
  }
}

function drawFluid(ctx: CanvasRenderingContext2D, w: World, cam: Camera): void {
  const f = w.fluid!
  const pts = f.points
  const rad = Math.max(1, f.pointRadius * cam.scale * 1.4)
  // draw as a soft metaball-ish blob: additive glow pass then core
  ctx.save()
  ctx.globalCompositeOperation = 'lighter'
  ctx.fillStyle = f.color
  for (const p of pts) {
    const s = cam.toScreen(p.x, p.y)
    ctx.beginPath()
    ctx.arc(s.x, s.y, rad, 0, Math.PI * 2)
    ctx.fill()
  }
  ctx.restore()
}

function drawParticles(ctx: CanvasRenderingContext2D, w: World, cam: Camera): void {
  const pts = w.particles.particles
  for (const p of pts) {
    const s = cam.toScreen(p.x, p.y)
    ctx.fillStyle = p.color
    const rad = Math.max(0.8, p.size * cam.scale)
    ctx.beginPath()
    ctx.arc(s.x, s.y, rad, 0, Math.PI * 2)
    ctx.fill()
  }
}

/** lighten (factor>1) or darken (factor<1) a #rrggbb hex colour. */
function shade(hex: string, factor: number): string {
  const n = parseInt(hex.slice(1), 16)
  let r = (n >> 16) & 255
  let g = (n >> 8) & 255
  let b = n & 255
  if (factor >= 1) {
    r = Math.min(255, r + (255 - r) * (factor - 1))
    g = Math.min(255, g + (255 - g) * (factor - 1))
    b = Math.min(255, b + (255 - b) * (factor - 1))
  } else {
    r *= factor
    g *= factor
    b *= factor
  }
  return `rgb(${r | 0},${g | 0},${b | 0})`
}
