/**
 * Viewport camera: maps between world coordinates (metres, Y-up) and screen
 * pixels (canvas, Y-down). Owns the pan (center) and zoom (pixels per metre).
 */
export class Camera {
  /** world-space centre of the viewport */
  x = 0
  y = 0
  /** pixels per metre */
  scale = 120
  /** canvas size in CSS pixels */
  w = 1
  h = 1

  resize(w: number, h: number): void {
    this.w = w
    this.h = h
  }

  /** world -> screen */
  toScreenX(wx: number): number {
    return (wx - this.x) * this.scale + this.w / 2
  }
  toScreenY(wy: number): number {
    return this.h / 2 - (wy - this.y) * this.scale
  }
  toScreen(wx: number, wy: number): { x: number; y: number } {
    return { x: this.toScreenX(wx), y: this.toScreenY(wy) }
  }

  /** screen -> world */
  toWorldX(sx: number): number {
    return (sx - this.w / 2) / this.scale + this.x
  }
  toWorldY(sy: number): number {
    return (this.h / 2 - sy) / this.scale + this.y
  }
  toWorld(sx: number, sy: number): { x: number; y: number } {
    return { x: this.toWorldX(sx), y: this.toWorldY(sy) }
  }

  /** pan by a screen-pixel delta */
  pan(dxScreen: number, dyScreen: number): void {
    this.x -= dxScreen / this.scale
    this.y += dyScreen / this.scale
  }

  /** zoom by a factor, keeping the world point under (sx, sy) fixed */
  zoomAt(factor: number, sx: number, sy: number): void {
    const before = this.toWorld(sx, sy)
    this.scale = Math.min(4000, Math.max(4, this.scale * factor))
    const after = this.toWorld(sx, sy)
    this.x += before.x - after.x
    this.y += before.y - after.y
  }

  zoomAtFactor(factor: number): void {
    this.zoomAt(factor, this.w / 2, this.h / 2)
  }

  /** fit a world rectangle (with padding) into the viewport */
  fit(minX: number, minY: number, maxX: number, maxY: number, pad = 0.15): void {
    const spanX = maxX - minX
    const spanY = maxY - minY
    if (spanX <= 0 || spanY <= 0) return
    this.scale = Math.min(4000, Math.max(4, Math.min(this.w / spanX, this.h / spanY) * (1 - pad)))
    this.x = (minX + maxX) / 2
    this.y = (minY + maxY) / 2
  }
}
