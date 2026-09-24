# Physics Sandbox

A browser-based 2D physics sandbox: a hand-written, dependency-free rigid-body
physics engine rendered in real time on Canvas2D, wrapped in a small React UI.
You can load a dozen preset scenes, grab and throw bodies, spawn new ones, wire
them together with rods and springs, scrub the simulation, and watch live
energy / momentum graphs — all running on a deterministic fixed-timestep core.

## Features

- **Custom 2D engine** — impulse-based contact solver (split-impulse with warm
  starting), continuous collision for fast bodies, sleeping, and a broadphase
  spatial hash. No physics library; every step is a pure function of the state.
- **Fixed 120 Hz timestep** decoupled from the render loop, with sub-step
  interpolation so the canvas stays smooth at 60 fps.
- **Two integrators** — velocity Verlet (default, 2nd-order symplectic) and
  semi-implicit Euler, switchable live to compare energy behaviour.
- **Gravity modes** — uniform field, mutual N-body point masses, or zero-g.
- **Soft bodies** — breakable PBD cloth, shape-matching blobs, and an SPH fluid
  tank (buoyancy is a known work-in-progress; see *Limitations*).
- **Interaction** — select, grab-and-throw, spawn, and link tools; pan / zoom;
  step / pause / reset; adjustable time scale (0–4×).
- **Instrumentation** — live kinetic / potential energy, linear and angular
  momentum, body count, and FPS.
- **Scene serialization** — full save / load round-trip of any scene to JSON.

## Presets

| Preset | What it shows |
| --- | --- |
| Blank / Blank (zero-g) | Empty canvas, optional zero gravity |
| Newton's Cradle | Momentum transfer through elastic collisions |
| Double Pendulum | Two near-identical pendulums diverging into chaos |
| Projectile Range | A live parabola |
| Inclined Plane & Friction | The slip threshold (µs vs tan θ) |
| Spring-Mass Oscillator | Simple harmonic motion |
| Collision Lab | Elastic collision, momentum & KE readouts |
| Orbital Mechanics | N-body sun / earth / moon + an eccentric comet |
| Galton Board | A binomial distribution from many small collisions |
| Cloth Drape | Breakable PBD cloth settling over a sphere |
| Fluid Tank | SPH water in a tank |
| Rube Goldberg | Starter pieces + the full toolbox |

## Running

```bash
npm install
npm run dev        # → http://localhost:5173/
```

Other scripts:

```bash
npm run build      # type-check (tsc) + bundle to dist/
npm run typecheck  # tsc --noEmit
npm run test       # vitest (engine + scene tests)
```

## Keyboard & mouse

- **Space** — play / pause
- **S** — single 120 Hz step
- **R** — reset (rebuild the current preset)
- **Del** — delete the selected body
- **Scroll** — zoom about the cursor; **middle-drag** or **Space+drag** — pan

## Project layout

```
src/
  engine/     the physics core — pure TS, no React, fully unit-tested
  ui/         renderer, camera, toolbar, inspector, graphs, status bar, audio
  App.tsx     the React component: owns the World + Camera, runs the sim loop
  main.tsx    entry point
test/         vitest suites (engine core + one per preset family)
```

See [`AGENTS.md`](./AGENTS.md) for the architecture in depth, including the
non-obvious invariants (split-impulse, velocity-gated warm starting, the
manifold normal convention, etc.) that keep the simulation stable.

## Testing

```bash
npm run test
npx vitest run test/engine.test.ts   # core physics only
npx vitest run -t "buoyancy"         # a single test by name
```

## Limitations

- **Fluid buoyancy** — the SPH water cannot yet produce emergent buoyancy at the
  120 Hz timestep (the lift pressure is impulsive and pushes bodies through the
  walls). The fluid is kept as a stable, non-buoyant feature; the corresponding
  scene test is documented as expected-to-fail until the fluid is reworked
  (e.g. Position-Based Fluids). 32/33 tests pass.

## Stack

React 19 · TypeScript (strict) · Vite · Tailwind CSS v4 · Canvas2D · Web Audio ·
Vitest. No physics engine — it is written from scratch in `src/engine/`.
