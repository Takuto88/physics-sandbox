import { useEffect, useRef, useState, useCallback } from 'react'
import { World } from './engine/world'
import type { Integrator } from './engine/world'
import { Body } from './engine/body'
import { circleShape, boxShape } from './engine/shapes'
import { MouseConstraint, DistanceConstraint, Constraint } from './engine/constraints'
import { PRESETS, findPreset } from './engine/presets'
import { Vec2 } from './engine/vec'
import { Camera } from './ui/camera'
import { render, RenderOptions, DEFAULT_RENDER_OPTS } from './ui/renderer'
import { ImpactAudio } from './ui/audio'
import { Toolbar } from './ui/Toolbar'
import { Inspector } from './ui/Inspector'
import { Graphs } from './ui/Graphs'
import { StatusBar } from './ui/StatusBar'

type Tool = 'select' | 'grab' | 'spawn' | 'constraint'

interface Hud {
  t: number
  bodies: number
  ke: number
  pe: number
  total: number
  px: number
  py: number
  L: number
  fps: number
  steps: number
}

export default function App() {
  const worldRef = useRef<World>(null!)
  if (!worldRef.current) {
    const w = findPreset('cradle')!.build()
    w.running = true
    worldRef.current = w
  }
  const cameraRef = useRef(new Camera())
  const canvasRef = useRef<HTMLCanvasElement | null>(null)
  const wrapRef = useRef<HTMLDivElement | null>(null)
  const audioRef = useRef(new ImpactAudio())
  const renderOptsRef = useRef<RenderOptions>({ ...DEFAULT_RENDER_OPTS })

  const rafRef = useRef(0)
  const lastTimeRef = useRef(0)
  const lastHudRef = useRef(0)
  const grabRef = useRef<MouseConstraint | null>(null)
  const mouseRef = useRef({ x: 0, y: 0, px: 0, py: 0, vx: 0, vy: 0 })
  const panningRef = useRef(false)
  const panStartRef = useRef({ x: 0, y: 0, cx: 0, cy: 0 })
  const constraintStartRef = useRef<number | null>(null)
  const spaceRef = useRef(false)

  const [presetId, setPresetId] = useState('cradle')
  const [running, setRunning] = useState(true)
  const [timeScale, setTimeScale] = useState(1)
  const [integrator, setIntegrator] = useState<Integrator>('verlet')
  const [gravityMode, setGravityMode] = useState<'uniform' | 'nbody' | 'none'>('uniform')
  const [tool, setTool] = useState<Tool>('select')
  const [spawnShape, setSpawnShape] = useState<'circle' | 'box'>('circle')
  const [selectedId, setSelectedId] = useState<number | null>(null)
  const [audioOn, setAudioOn] = useState(false)
  const [renderOpts, setRenderOpts] = useState<RenderOptions>({ ...DEFAULT_RENDER_OPTS })
  const [hud, setHud] = useState<Hud>({ t: 0, bodies: 0, ke: 0, pe: 0, total: 0, px: 0, py: 0, L: 0, fps: 0, steps: 0 })

  // keep the render-options ref in sync for the rAF loop
  renderOptsRef.current = renderOpts
  renderOptsRef.current.selectedId = selectedId

  const world = worldRef.current

  // ---- fit the camera to the world's content bounds -----------------------
  const fitCamera = useCallback((w: World) => {
    let minX = -4, minY = -3, maxX = 4, maxY = 3
    if (w.bodies.length === 0) {
      cameraRef.current.fit(minX, minY, maxX, maxY)
      return
    }
    for (const b of w.bodies) {
      if (b.kind === 'static') continue
      b.updateAABB()
      minX = Math.min(minX, b.aabb.minX)
      minY = Math.min(minY, b.aabb.minY)
      maxX = Math.max(maxX, b.aabb.maxX)
      maxY = Math.max(maxY, b.aabb.maxY)
    }
    cameraRef.current.fit(minX, minY, maxX, maxY)
  }, [])

  // ---- build a fresh world from a preset ---------------------------------
  const loadPreset = useCallback(
    (id: string) => {
      const preset = findPreset(id)
      if (!preset) return
      const w = preset.build()
      w.config.timeScale = worldRef.current.config.timeScale
      w.config.integrator = worldRef.current.config.integrator
      w.running = true
      w.onContact = (e) => {
        if (audioRef.current.enabled) audioRef.current.impact(e.impulse, e.x, e.y)
      }
      worldRef.current = w
      setPresetId(id)
      setTimeScale(w.config.timeScale)
      setIntegrator(w.config.integrator)
      setGravityMode(w.gravity.mode)
      setRunning(true)
      setSelectedId(null)
      grabRef.current = null
      constraintStartRef.current = null
      fitCamera(w)
    },
    [fitCamera],
  )

  // ---- canvas sizing (device-pixel-ratio aware) ---------------------------
  useEffect(() => {
    const canvas = canvasRef.current!
    const wrap = wrapRef.current!
    const resize = () => {
      const dpr = window.devicePixelRatio || 1
      const r = wrap.getBoundingClientRect()
      canvas.width = Math.round(r.width * dpr)
      canvas.height = Math.round(r.height * dpr)
      canvas.style.width = r.width + 'px'
      canvas.style.height = r.height + 'px'
      cameraRef.current.resize(r.width, r.height)
    }
    resize()
    const ro = new ResizeObserver(resize)
    ro.observe(wrap)
    return () => ro.disconnect()
  }, [])

  // ---- fit camera on first mount (sized canvas) ---------------------------
  useEffect(() => {
    fitCamera(worldRef.current)
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [])

  // ---- wire impact audio to the initial world (loadPreset re-wires on
  //      preset changes) ------------------------------------------------------
  useEffect(() => {
    const w = worldRef.current
    w.onContact = (e) => {
      if (audioRef.current.enabled) audioRef.current.impact(e.impulse, e.x, e.y)
    }
  }, [])

  // ---- main simulation + render loop --------------------------------------
  useEffect(() => {
    const loop = (now: number) => {
      const dt = Math.min(0.05, (now - lastTimeRef.current) / 1000)
      lastTimeRef.current = now
      const w = worldRef.current
      const cam = cameraRef.current

      // pointer velocity for the grab tool
      const m = mouseRef.current
      if (dt > 0) {
        m.vx = (m.x - m.px) / dt
        m.vy = (m.y - m.py) / dt
        m.px = m.x
        m.py = m.y
      }
      if (grabRef.current) {
        grabRef.current.target.set(m.x, m.y)
        grabRef.current.targetVel.set(m.vx, m.vy)
      }

      if (w.running) w.tick(dt)

      const canvas = canvasRef.current
      const ctx = canvas?.getContext('2d')
      if (canvas && ctx) {
        const dpr = window.devicePixelRatio || 1
        render(ctx, w, cam, renderOptsRef.current, dpr)
      }

      // HUD at ~12 Hz
      if (now - lastHudRef.current > 80) {
        lastHudRef.current = now
        const e = w.energy
        setHud({
          t: w.time,
          bodies: w.bodies.length,
          ke: e.ke,
          pe: e.pe,
          total: e.total,
          px: e.px,
          py: e.py,
          L: e.L,
          fps: dt > 0 ? 1 / dt : 0,
          steps: w.stepsThisTick,
        })
      }
      rafRef.current = requestAnimationFrame(loop)
    }
    lastTimeRef.current = performance.now()
    rafRef.current = requestAnimationFrame(loop)
    return () => cancelAnimationFrame(rafRef.current)
  }, [])

  // ---- keyboard: space = play/pause, R = reset, S = step ------------------
  useEffect(() => {
    const down = (e: KeyboardEvent) => {
      if (e.code === 'Space') {
        e.preventDefault()
        spaceRef.current = true
        setRunning((r) => {
          worldRef.current.running = !r
          return !r
        })
      } else if (e.key === 'r' || e.key === 'R') {
        loadPreset(presetId)
      } else if (e.key === 's' || e.key === 'S') {
        setRunning(false)
        worldRef.current.running = false
        worldRef.current.singleStep()
      } else if (e.key === 'Delete' || e.key === 'Backspace') {
        if (selectedId != null) {
          const b = worldRef.current.bodies.find((x) => x.id === selectedId)
          if (b) {
            worldRef.current.removeBody(b)
            setSelectedId(null)
          }
        }
      }
    }
    const up = (e: KeyboardEvent) => {
      if (e.code === 'Space') spaceRef.current = false
    }
    window.addEventListener('keydown', down)
    window.addEventListener('keyup', up)
    return () => {
      window.removeEventListener('keydown', down)
      window.removeEventListener('keyup', up)
    }
  }, [presetId, selectedId, loadPreset])

  // ---- sync UI controls to the live world ---------------------------------
  const applyControls = useCallback((w: World) => {
    w.running = running
    w.config.timeScale = timeScale
    w.config.integrator = integrator
  }, [running, timeScale, integrator])

  useEffect(() => {
    applyControls(worldRef.current)
  }, [applyControls])

  const setGravity = useCallback((mode: 'uniform' | 'nbody' | 'none') => {
    const w = worldRef.current
    w.gravity.mode = mode
    w.config.gravityMode = mode
    w.syncGravity()
    w.wakeAll()
    setGravityMode(mode)
  }, [])

  // ---- coordinate helpers -------------------------------------------------
  const toWorld = useCallback((clientX: number, clientY: number) => {
    const canvas = canvasRef.current!
    const rect = canvas.getBoundingClientRect()
    return cameraRef.current.toWorld(clientX - rect.left, clientY - rect.top)
  }, [])

  // ---- pointer handlers ---------------------------------------------------
  const onPointerDown = useCallback(
    (e: React.PointerEvent) => {
      audioRef.current.unlock()
      ;(e.target as Element).setPointerCapture(e.pointerId)
      const w = worldRef.current
      const cam = cameraRef.current
      const wp = toWorld(e.clientX, e.clientY)
      const m = mouseRef.current
      m.x = wp.x
      m.y = wp.y
      m.px = wp.x
      m.py = wp.y

      const rect = canvasRef.current!.getBoundingClientRect()
      const sx = e.clientX - rect.left
      const sy = e.clientY - rect.top
      const middle = e.button === 1
      const wantPan = middle || spaceRef.current

      if (wantPan) {
        panningRef.current = true
        panStartRef.current = { x: sx, y: sy, cx: cam.x, cy: cam.y }
        return
      }

      const picked = w.pickPoint(new Vec2(wp.x, wp.y))

      if (tool === 'select') {
        setSelectedId(picked ? picked.id : null)
        if (!picked) {
          panningRef.current = true
          panStartRef.current = { x: sx, y: sy, cx: cam.x, cy: cam.y }
        }
      } else if (tool === 'grab') {
        setSelectedId(picked ? picked.id : null)
        if (picked && picked.isDynamic()) {
          const anchor = Constraint.localOf(picked, new Vec2(wp.x, wp.y))
          const mc = new MouseConstraint(picked, anchor, new Vec2(wp.x, wp.y))
          w.addConstraint(mc)
          grabRef.current = mc
        } else if (!picked) {
          panningRef.current = true
          panStartRef.current = { x: sx, y: sy, cx: cam.x, cy: cam.y }
        }
      } else if (tool === 'spawn') {
        const mat = { restitution: 0.4, friction: 0.5, density: 1, color: '#7aa2ff', glow: false }
        const shape = spawnShape === 'circle' ? circleShape(0.3) : boxShape(0.5, 0.5)
        const b = new Body(shape, mat)
        b.pos.set(wp.x, wp.y)
        w.addBody(b)
        setSelectedId(b.id)
      } else if (tool === 'constraint') {
        if (constraintStartRef.current == null) {
          constraintStartRef.current = picked ? picked.id : null
          setSelectedId(picked ? picked.id : null)
        } else {
          const a = w.bodies.find((x) => x.id === constraintStartRef.current)
          const b = picked
          if (a && b && a !== b) {
            // connect the two centres with a rigid rod at the current spacing
            w.addConstraint(new DistanceConstraint(a, b, new Vec2(0, 0), new Vec2(0, 0), a.pos.dist(b.pos)))
          }
          constraintStartRef.current = null
        }
      }
    },
    [tool, spawnShape, toWorld],
  )

  const onPointerMove = useCallback(
    (e: React.PointerEvent) => {
      const cam = cameraRef.current
      const wp = toWorld(e.clientX, e.clientY)
      const m = mouseRef.current
      m.x = wp.x
      m.y = wp.y
      if (panningRef.current) {
        const rect = canvasRef.current!.getBoundingClientRect()
        const sx = e.clientX - rect.left
        const sy = e.clientY - rect.top
        const s = panStartRef.current
        cam.x = s.cx - (sx - s.x) / cam.scale
        cam.y = s.cy + (sy - s.y) / cam.scale
      }
    },
    [toWorld],
  )

  const onPointerUp = useCallback((e: React.PointerEvent) => {
    ;(e.target as Element).releasePointerCapture(e.pointerId)
    panningRef.current = false
    if (grabRef.current) {
      worldRef.current.removeConstraint(grabRef.current)
      grabRef.current = null
    }
  }, [])

  const onWheel = useCallback((e: React.WheelEvent) => {
    const cam = cameraRef.current
    const rect = canvasRef.current!.getBoundingClientRect()
    const sx = e.clientX - rect.left
    const sy = e.clientY - rect.top
    const factor = Math.exp(-e.deltaY * 0.0012)
    cam.zoomAt(factor, sx, sy)
  }, [])

  // ---- render -------------------------------------------------------------
  return (
    <div className="h-full flex flex-col text-slate-200">
      <Toolbar
        presets={PRESETS}
        presetId={presetId}
        onPreset={loadPreset}
        running={running}
        onToggleRunning={() => setRunning((r) => !r)}
        onStep={() => {
          worldRef.current.running = false
          setRunning(false)
          worldRef.current.singleStep()
        }}
        onReset={() => loadPreset(presetId)}
        timeScale={timeScale}
        onTimeScale={setTimeScale}
        integrator={integrator}
        onIntegrator={setIntegrator}
        gravityMode={gravityMode}
        onGravityMode={setGravity}
        tool={tool}
        onTool={setTool}
        spawnShape={spawnShape}
        onSpawnShape={setSpawnShape}
        renderOpts={renderOpts}
        onRenderOpts={setRenderOpts}
        audioOn={audioOn}
        onAudioToggle={() => {
          const next = !audioOn
          setAudioOn(next)
          audioRef.current.enabled = next
          if (next) audioRef.current.unlock()
        }}
      />
      <div className="flex-1 flex min-h-0">
        <div ref={wrapRef} className="flex-1 relative min-w-0 select-none">
          <canvas
            ref={canvasRef}
            className="block touch-none"
            onPointerDown={onPointerDown}
            onPointerMove={onPointerMove}
            onPointerUp={onPointerUp}
            onPointerCancel={onPointerUp}
            onWheel={onWheel}
          />
        </div>
        <aside className="w-72 shrink-0 border-l border-slate-700/60 flex flex-col min-h-0">
          <Inspector world={world} selectedId={selectedId} onSelect={setSelectedId} version={hud.t} />
          <Graphs history={world.history} version={hud.t} />
        </aside>
      </div>
      <StatusBar hud={hud} presetName={findPreset(presetId)?.name ?? ''} />
    </div>
  )
}
