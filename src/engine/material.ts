export interface Material {
  /** 0..1 (1 = perfectly elastic). Pairs combine with max(). */
  restitution: number
  /** Coulomb coefficient, static and kinetic. Pairs combine with sqrt(). */
  friction: number
  /** kg/m^2 (2D area density). Mass and inertia are derived, never entered. */
  density: number
  color: string
  /** high-restitution materials get a soft glow in the renderer */
  glow: boolean
}

export const DEFAULT_MATERIAL: Material = {
  restitution: 0.35,
  friction: 0.4,
  density: 1,
  color: '#7aa2ff',
  glow: false,
}

export function combineRestitution(a: Material, b: Material): number {
  return Math.max(a.restitution, b.restitution)
}

export function combineFriction(a: Material, b: Material): number {
  return Math.sqrt(a.friction * b.friction)
}
