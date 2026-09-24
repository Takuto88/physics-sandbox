# AGENTS.md — Physics Sandbox

A browser-based 2D physics sandbox. A hand-written, dependency-free physics engine
(`src/engine/`, framework-agnostic, fully unit-tested) rendered on Canvas2D through a
React UI (`src/ui/`, `src/App.tsx`). Correctness of the physics is the top priority,
ahead of interactivity, visualization, and breadth.

This file is the single source of truth for an agent working here. Read it before
changing the engine. The "do-not-break" section lists invariants that are not obvious
from reading a single file.

---

## Commands

```
npm run dev          # vite dev server → http://localhost:5173/  (port is fixed in vite.config.ts)
npm run build        # tsc --noEmit && vite build   (type-checks, then bundles to dist/)
npm run typecheck    # tsc --noEmit
npm run test         # vitest run                   (test/**/*.test.ts, node environment)
npx vitest run test/engine.test.ts          # engine core tests only
npx vitest run -t "buoyancy"               # run a single test by name
```

There is no git repo here. There is no lint script; the "linter" is `tsc` in strict
mode (see Gotchas).

---

## Project layout

```
index.html                  mounts #root, loads /src/main.tsx
vite.config.ts              react + @tailwindcss/vite plugins; vitest: environment 'node', test/**
tsconfig.json               strict, noUnusedLocals, noUnusedParameters, noFallthroughCasesInSwitch

src/
  main.tsx                  React entry
  App.tsx                   THE component: owns the World + Camera, runs the rAF sim loop,
                            handles all canvas pointer/keyboard input, wires the panels
  index.css                 Tailwind v4 import + full-height dark base

  engine/                   PURE TS. No React, no DOM. This is what the tests exercise.
    world.ts                World: the sim core. fixed-timestep tick/step, the per-substep
                            pipeline, instrumentation (energy/history/snapshots), scrub,
                            pickPoint, sleep bookkeeping. Exports DT, PHYSICS_HZ, Integrator.
    body.ts                 Body: pose/velocity/shape/material/mass; setKind, localToWorld,
                            applyImpulse, updateAABB. prevPos/prevAngle feed render interpolation.
    shapes.ts               Shape = Circle | Polygon (a box is a 4-vert polygon — there is no
                            box type). circleShape / boxShape / hullShape / massProperties.
    vec.ts                  Vec2 (mutable, chaining) + free helpers.
    material.ts             Material {restitution,friction,density,color,glow}; combine rules.
    contact.ts              ContactPoint / Manifold data structures.
    narrowphase.ts          generateManifold(a,b): circle/circle, circle/polygon, polygon/polygon
                            (SAT + Sutherland-Hodgman), GJK+EPA for hulls.
    broadphase.ts           uniform spatial-hash grid; collect(bodies, fn).
    solver.ts               sequential-impulse contact solver WITH warm starting (see invariants).
    ccd.ts                  swept-circle continuous collision for small fast bodies.
    constraints.ts          Constraint subclasses: Distance, Rope, Revolute, Prismatic, Weld,
                            Spring, Mouse (live grab). anchorA()/anchorB() give world endpoints.
    gravity.ts              Gravity {mode, field, G, softening}; bodyMass() (see invariant).
    particles.ts            ParticleSystem: sand/confetti/spark/dust (NOT cloth).
    softbody.ts             Cloth (PBD grid, breakable) + ShapeMatchingBlob.
    fluid.ts                Clavet double-density SPH (see Known limitations).
    scene.ts                worldToScene / sceneToWorld (JSON round-trip) + makeCircle/makeBox.
    presets.ts              PRESETS[] (13) + findPreset(id). Each build() returns a fresh World.
    rand.ts                 deterministic mulberry32 RNG (reproducible scenes).
    collidePoint.ts         shared point-vs-body push-out used by cloth/blob/sand/fluid.

  ui/
    renderer.ts             Canvas2D draw of the whole world in screen space (see rendering).
    camera.ts               Camera: world↔screen transform, pan/zoom/fit.
    Toolbar.tsx             presets, transport, time scale, integrator, gravity, tools, toggles.
    Inspector.tsx           edit selected body (pose/vel/shape/material) + scene stats.
    Graphs.tsx              live energy + momentum line plots (reads world.history).
    StatusBar.tsx           fps / body count / KE / PE / momentum HUD.
    audio.ts                ImpactAudio: Web Audio noise-burst synth, driven by world.onContact.

test/
  engine.test.ts            23 core physics tests (solver, manifolds, integrators, sleep, …)
  scenes.test.ts            10 preset tests (cradle, pendulum, orbital, galton, fluid, …)
```

---

## The physics core — invariants (READ BEFORE EDITING)

### Fixed timestep, decoupled from rendering
- `PHYSICS_HZ = 120`, `DT = 1/120` (`world.ts:18-19`). The physics always steps in
  exact `DT` increments. Never step with the raw frame delta — that destroys
  determinism and breaks scrub (the whole reason scrub exists is that the step is a
  pure function of `(state, DT)`).
- The render loop calls `world.tick(realDt)` once per animation frame. `tick` adds
  `realDt * timeScale * baseTimeScale` to an accumulator and runs up to 8 `DT` steps
  (spiral-of-death guard), then sets `world.alpha = accumulator / DT`.
- `world.alpha` + `body.prevPos`/`body.prevAngle` are for render interpolation: draw
  `prev + (current - prev) * alpha`. The renderer already does this — keep
  `prevPos`/`prevAngle` updated every step.

### Split-impulse (this is deliberate, do not "simplify")
- The **velocity solve** (`solver.solve`) uses `baumgarte: 0` — it handles restitution
  and friction only. Penetration recovery is a **separate projection pass** in the
  world (`world.ts` position-projection, percent 0.5, applied once at max penetration).
  Merging position correction back into the velocity solve re-introduces the jitter
  that split-impulse was chosen to remove.
- `solverIterations = 10`. Raising it a lot (e.g. 40) makes some scenes (Galton board)
  explode regardless of warm-start; 10 is tuned.

### Warm starting is velocity-gated (subtle, easily "broken")
- `solver.ts` only warm-starts a contact when **both** bodies are slow
  (`vel.len() < WARM_SPEED = 0.5`). Re-applying the previous step's accumulated normal
  impulse unconditionally injects energy: the one-sided clamp `jn ≥ 0` can only ADD
  impulse, so a stale bounce re-applied to a separating pair pumps it up forever.
  If a scene starts gaining energy out of thin air, suspect this gate.
- The warm normal impulse is also clamped to `jn ≤ -vn0 * kN` (it may cancel approach,
  never create separation), and tangent to the Coulomb cone.

### Contact / manifold convention
- The manifold normal always points **a → b**. In the solver, `vn < 0` means the pair is
  approaching. Don't flip the convention — narrowphase, solver, and projection all assume it.

### Gravity
- Modes: `uniform` (constant field), `nbody` (mutual point masses), `none`.
- In `nbody` mode the gravitational mass is `body.userData['gravMass']`;
  `gravity.bodyMass()` reads that for **all** bodies (falling back to inertial mass).
  This was a real bug fix — do not revert it to "static bodies only".
- After changing gravity from the UI: set `world.gravity.*` or `world.config.*` then
  `world.syncGravity()`, and `world.wakeAll()` (sleeping bodies don't feel gravity).

### Sleep
- A dynamic body that is slow AND in contact (or zero-gravity) for 0.5 s sleeps
  (`sleeping = true`, velocity zeroed, `effInvMass/effInvInertia → 0`). It is woken by
  a strong impact. Sleeping bodies must report zero effective mass everywhere (solver,
  gravity, constraints) or stacks creep.

### Contact events (sound)
- Use `world.onContact(cb)` — the reliable per-contact hook. `world.contactEvents` is
  cleared at the end of **every substep**, so polling it after `tick()` returns always
  yields an empty array.

### Broadphase
- Spatial-hash grid; `collect(bodies, fn)` clears its pair set every call. Pairs are
  unique (a<b). If you add bodies mid-frame, the AABBs must be current (`updateAABB`).

---

## Rendering & the React/engine split

- The engine knows nothing about React or the DOM. `App.tsx` is the only bridge.
- The World lives in a **ref** (`worldRef`), not React state — the rAF loop reads
  `worldRef.current` every frame so React re-renders never rebuild or stall the sim.
  React re-renders happen at ~12 Hz (the HUD `setHud`), decoupled from the 60 fps canvas.
- Mirroring state into refs for the loop: `renderOptsRef` is assigned during render and
  read by the loop. Keep that pattern for any per-frame parameter.
- Preset switching = `findPreset(id).build()` into a fresh `World`; there is no
  "reset to preset" method on `World` (`clearWorld()` empties in place instead).
- **Coordinate systems:** world is metres, Y-up; the canvas is pixels, Y-down. The
  renderer maps every point through `Camera` (`toScreenX/toScreenY`) in screen space
  rather than using one `ctx` transform — that keeps line widths, text, and the Y-flip
  correct. `Camera` owns `x,y` (center) and `scale` (px per metre).
- Colors live on `body.material.color` (not on `Body`); `material.glow` adds a shadow
  glow. Fluid is drawn with `globalCompositeOperation='lighter'` (additive metaball look).

### Tools (in App.tsx pointer handlers)
- **select** — click to select (drives the Inspector); drag empty space to pan.
- **grab** — a `MouseConstraint` pulls the grabbed body's anchor to the cursor with a
  pointer-velocity term, so releasing throws it. Remove the constraint on pointer-up.
- **spawn** — drop a circle/box at the cursor.
- **link** — click two bodies to add a rigid `DistanceConstraint` between their centers.
- Middle-mouse or Space+drag always pans; wheel zooms about the cursor.
- Keyboard: Space = play/pause, S = single step, R = reset (rebuild preset), Del = delete selection.

---

## Testing

- Framework: **vitest**, node environment, `test/**/*.test.ts`. Tests import the engine
  directly and step worlds with the exported `DT`.
- `test/engine.test.ts` (23 tests): solver, manifold generation, integrators,
  restitution/friction, sleep, constraints, gravity, scrub, save/load round-trip.
- `test/scenes.test.ts` (10 tests): one per preset family, asserting emergent behavior
  (e.g. "moon stays in its Hill-stable orbit", "first cradle ball swings out").
- A scratch file `test/debug.test.ts` may be created while diagnosing — **delete it
  before finishing**. It is not part of the real suite.
- Scene tests run many steps (`120 * N`), so they take a few seconds; that's expected.
- **Expected-to-fail:** `fluid tank (T8)` — see Known limitations. 32/33 pass is the
  correct, intended baseline. Don't "fix" the fluid to make T8 green without reworking
  the SPH (see below); the test documents the intended behavior.

---

## Known limitations

### Fluid / buoyancy (T8) — accepted, 2026-09
- The Clavet double-density SPH cannot produce emergent buoyancy at the 120 Hz timestep.
  The pressure that *would* lift a body is impulsive and drives the rigid boxes through
  the tank walls; the stable (soft) regime has too little pressure to float anything.
- `DEFAULT_FLUID.forceScale = 0` (and `nearForceScale = 0`) deliberately decouples
  particle pressure from rigid bodies: the fluid still collides (position projection)
  but exerts no buoyant force, so the boxes settle instead of tunneling.
- Stability fixes that ARE in `fluid.ts` and must stay: per-pair displacement clamp
  (`MAX_DISPLACEMENT`), a particle speed cap (`MAX_SPEED_SQ`) so nothing tunnels a wall,
  inelastic boundary bounce, and re-deriving velocity *before* the boundary projection.
- The fluid still slowly leaks through the floor over long runs (a multi-particle
  pressure effect, not a collision bug — a single particle is contained correctly).
- To actually get buoyancy later: rework with Position-Based Fluids (PBF) or much
  higher effective substepping, then re-enable `forceScale` and remove the T8 caveat.

---

## Gotchas / conventions

- **tsc strict** with `noUnusedLocals` and `noUnusedParameters`. An unused import or
  variable fails the build/typecheck. `npm run build` runs `tsc` first, so a stray
  import breaks the build.
- `isolatedModules` is on — re-export types with `export type { … }`.
- The engine is deterministic for a given `world.rng` seed (mulberry32). Don't call
  `Math.random()` in the engine; use `world.rng` / `new RNG(seed)`.
- Shapes are either a circle or a polygon; a "box" is `boxShape(w,h)` returning a
  4-vertex polygon. There is no `type:'box'` and no `hw/hh` — don't assume them.
- Mutating a body for the Inspector: set the field, then `recomputeMass()` (if
  shape/density/kind changed) and `updateAABB()`; `wake()` if you changed velocity.
- `three` is a dependency but currently unused (the 2D top-down view renders orbital
  mechanics fine). Only add a Three.js view if/when it's actually needed.
- Scene save/load (`worldToScene`/`sceneToWorld`) exists in `engine/scene.ts` and is
  unit-tested, but the UI does not yet expose save/load buttons — it's available-but-unwired.
- `dist/` is build output; don't edit it.

---

## Where things are (quick lookup)

- Start the sim / step it: `world.tick(dt)`, `world.singleStep()`, `world.step(dt)` — `world.ts`.
- Add/remove a body: `world.addBody / removeBody` (`world.ts`).
- Selection from a screen point: `world.pickPoint(worldVec)` (`world.ts`).
- Draw everything: `render(ctx, world, camera, opts, dpr)` (`ui/renderer.ts`).
- World↔screen: `camera.toWorld / toScreen / zoomAt / fit` (`ui/camera.ts`).
- Load a scene: `findPreset(id).build()` (`engine/presets.ts`).
- Serialize: `worldToScene / sceneToWorld` (`engine/scene.ts`).
