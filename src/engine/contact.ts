import { Vec2 } from './vec'
import { Body } from './body'

/**
 * One contact point in a manifold. The normal always points from body A to
 * body B. `id` is a stable feature key (edge/vertex indices) used to match
 * warm-start impulses across frames.
 */
export interface ContactPoint {
  pos: Vec2
  normal: Vec2
  tangent: Vec2
  penetration: number
  /** incoming relative normal velocity (pre-solve), negative = approaching */
  vn0: number
  restitution: number
  friction: number
  id: number
  // accumulated solver state (persists across iterations and frames)
  normalImpulse: number
  tangentImpulse: number
}

export interface Manifold {
  a: Body
  b: Body
  points: ContactPoint[]
}

export function makeContact(
  pos: Vec2,
  normal: Vec2,
  penetration: number,
  restitution: number,
  friction: number,
  id: number,
): ContactPoint {
  return {
    pos: pos.clone(),
    normal: normal.clone(),
    tangent: new Vec2(-normal.y, normal.x),
    penetration,
    vn0: 0,
    restitution,
    friction,
    id,
    normalImpulse: 0,
    tangentImpulse: 0,
  }
}
