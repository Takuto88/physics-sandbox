/**
 * Vec2 — minimal 2D vector. Hot solver loops use raw numbers; this class is
 * for the API surface. All mutating ops return `this` for chaining.
 */
export class Vec2 {
  constructor(public x = 0, public y = 0) {}

  set(x: number, y: number): this {
    this.x = x
    this.y = y
    return this
  }

  copy(v: Vec2): this {
    this.x = v.x
    this.y = v.y
    return this
  }

  clone(): Vec2 {
    return new Vec2(this.x, this.y)
  }

  add(v: Vec2): this {
    this.x += v.x
    this.y += v.y
    return this
  }

  sub(v: Vec2): this {
    this.x -= v.x
    this.y -= v.y
    return this
  }

  scale(s: number): this {
    this.x *= s
    this.y *= s
    return this
  }

  addScaled(v: Vec2, s: number): this {
    this.x += v.x * s
    this.y += v.y * s
    return this
  }

  len(): number {
    return Math.hypot(this.x, this.y)
  }

  lenSq(): number {
    return this.x * this.x + this.y * this.y
  }

  normalize(): this {
    const l = this.len()
    if (l > 1e-12) {
      this.x /= l
      this.y /= l
    }
    return this
  }

  dot(v: Vec2): number {
    return this.x * v.x + this.y * v.y
  }

  /** 2D cross (returns the z component). */
  cross(v: Vec2): number {
    return this.x * v.y - this.y * v.x
  }

  /** Cross of a scalar with the vector: s * perp(v). */
  crossSV(s: number): Vec2 {
    return new Vec2(-s * this.y, s * this.x)
  }

  dist(v: Vec2): number {
    const dx = this.x - v.x
    const dy = this.y - v.y
    return Math.hypot(dx, dy)
  }

  negate(): this {
    this.x = -this.x
    this.y = -this.y
    return this
  }

  static fromAngle(a: number, len = 1): Vec2 {
    return new Vec2(Math.cos(a) * len, Math.sin(a) * len)
  }
}

/** World-space cross: a x b for position-velocity terms used in the solver. */
export function crossVV(a: Vec2, b: Vec2): number {
  return a.x * b.y - a.y * b.x
}

/** a x v (scalar x vector -> vector). */
export function crossSV(s: number, v: Vec2): Vec2 {
  return new Vec2(-s * v.y, s * v.x)
}
