import { World } from '../engine/world'
import { Body } from '../engine/body'
import { circleShape } from '../engine/shapes'

interface Props {
  world: World
  selectedId: number | null
  onSelect: (id: number | null) => void
  version: number
}

function Num({
  label,
  value,
  step = 0.1,
  onChange,
}: {
  label: string
  value: number
  step?: number
  onChange: (v: number) => void
}) {
  return (
    <label className="flex items-center gap-2 text-xs">
      <span className="w-16 text-slate-400 shrink-0">{label}</span>
      <input
        type="number"
        step={step}
        value={Number.isFinite(value) ? Number(value.toFixed(3)) : 0}
        onChange={(e) => onChange(parseFloat(e.target.value) || 0)}
        className="flex-1 min-w-0 h-6 rounded bg-slate-800/70 border border-slate-700 px-1.5 text-slate-200"
      />
    </label>
  )
}

function Section({ title, children }: { title: string; children: React.ReactNode }) {
  return (
    <div className="border-t border-slate-700/50 pt-2 mt-2">
      <div className="text-[10px] uppercase tracking-wider text-slate-500 mb-1.5">{title}</div>
      <div className="space-y-1.5">{children}</div>
    </div>
  )
}

export function Inspector({ world, selectedId, onSelect, version }: Props) {
  void version
  const body = selectedId != null ? world.bodies.find((b) => b.id === selectedId) ?? null : null

  const mutate = (fn: (b: Body) => void) => {
    if (!body) return
    fn(body)
    if (body.isDynamic()) body.wake()
    body.updateAABB()
  }

  return (
    <div className="flex-1 overflow-y-auto p-3 min-h-0">
      <div className="flex items-center justify-between mb-1">
        <span className="text-xs font-semibold text-slate-300">Inspector</span>
        {body && (
          <button onClick={() => onSelect(null)} className="text-[11px] text-slate-500 hover:text-slate-300">
            deselect
          </button>
        )}
      </div>

      {!body ? (
        <SceneInfo world={world} />
      ) : (
        <>
          <div className="text-[11px] text-slate-400">
            <span className="text-slate-300 font-medium">
              {body.label || (body.shape.type === 'circle' ? 'circle' : 'polygon')}
            </span>{' '}
            · #{body.id} · {body.kind}
          </div>

          <Section title="Body">
            <div className="flex gap-1">
              {(['dynamic', 'static', 'kinematic'] as const).map((k) => (
                <button
                  key={k}
                  onClick={() => mutate((b) => b.setKind(k))}
                  className={
                    'flex-1 h-6 rounded text-[11px] border ' +
                    (body.kind === k
                      ? 'bg-sky-500/25 border-sky-400/60 text-sky-200'
                      : 'bg-slate-800/70 border-slate-700 text-slate-400 hover:bg-slate-700/70')
                  }
                >
                  {k}
                </button>
              ))}
            </div>
            <label className="flex items-center gap-2 text-xs">
              <span className="w-16 text-slate-400">Label</span>
              <input
                type="text"
                value={body.label}
                onChange={(e) => mutate((b) => (b.label = e.target.value))}
                className="flex-1 min-w-0 h-6 rounded bg-slate-800/70 border border-slate-700 px-1.5 text-slate-200"
              />
            </label>
          </Section>

          <Section title="Transform">
            <Num label="X" value={body.pos.x} step={0.1} onChange={(v) => mutate((b) => b.pos.set(v, b.pos.y))} />
            <Num label="Y" value={body.pos.y} step={0.1} onChange={(v) => mutate((b) => b.pos.set(b.pos.x, v))} />
            <Num
              label="Angle°"
              value={(body.angle * 180) / Math.PI}
              step={5}
              onChange={(v) => mutate((b) => (b.angle = (v * Math.PI) / 180))}
            />
            <Num label="VX" value={body.vel.x} step={0.5} onChange={(v) => mutate((b) => b.vel.set(v, b.vel.y))} />
            <Num label="VY" value={body.vel.y} step={0.5} onChange={(v) => mutate((b) => b.vel.set(b.vel.x, v))} />
            <Num label="AngV" value={body.angVel} step={0.5} onChange={(v) => mutate((b) => (b.angVel = v))} />
          </Section>

          {body.shape.type === 'circle' ? (
            <Section title="Shape · circle">
              <Num
                label="Radius"
                value={body.shape.radius}
                step={0.05}
                onChange={(v) => mutate((b) => (b.shape = circleShape(Math.max(0.02, v))))}
              />
            </Section>
          ) : (
            <Section title="Shape · polygon">
              <div className="text-[11px] text-slate-400">{body.shape.verts.length} vertices</div>
            </Section>
          )}

          <Section title="Material">
            <Num
              label="Restit."
              value={body.material.restitution}
              step={0.05}
              onChange={(v) => mutate((b) => (b.material.restitution = Math.min(1, Math.max(0, v))))}
            />
            <Num
              label="Friction"
              value={body.material.friction}
              step={0.05}
              onChange={(v) => mutate((b) => (b.material.friction = Math.max(0, v)))}
            />
            <Num
              label="Density"
              value={body.material.density}
              step={0.5}
              onChange={(v) => mutate((b) => (b.material.density = Math.max(0.01, v)))}
            />
            <label className="flex items-center gap-2 text-xs">
              <span className="w-16 text-slate-400">Colour</span>
              <input
                type="color"
                value={body.material.color}
                onChange={(e) => mutate((b) => (b.material.color = e.target.value))}
                className="h-6 w-10 rounded bg-slate-800/70 border border-slate-700"
              />
            </label>
            <label className="flex items-center gap-2 text-xs text-slate-400">
              <input
                type="checkbox"
                checked={body.material.glow}
                onChange={(e) => mutate((b) => (b.material.glow = e.target.checked))}
                className="accent-sky-400"
              />
              Glow
            </label>
          </Section>

          <div className="mt-3 grid grid-cols-2 gap-1 text-[11px] text-slate-400">
            <div>mass {body.mass.toFixed(3)}</div>
            <div>inertia {body.inertia.toFixed(3)}</div>
            <div>|v| {body.vel.len().toFixed(2)}</div>
            <div>{body.sleeping ? 'asleep' : 'awake'}</div>
          </div>

          <button
            onClick={() => {
              world.removeBody(body)
              onSelect(null)
            }}
            className="mt-3 w-full h-7 rounded text-xs font-medium bg-rose-500/15 border border-rose-500/40 text-rose-300 hover:bg-rose-500/25"
          >
            Delete body
          </button>
        </>
      )}
    </div>
  )
}

function SceneInfo({ world }: { world: World }) {
  const rows: [string, string][] = [
    ['Bodies', String(world.bodies.length)],
    ['Constraints', String(world.constraints.length)],
    ['Cloths', String(world.cloths.length)],
    ['Blobs', String(world.blobs.length)],
    ['Fluid', world.fluid ? `${world.fluid.points.length} pts` : '—'],
    ['Particles', String(world.particles.particles.length)],
    ['Emitters', String(world.emitters.length)],
    ['Substeps', String(world.config.substeps)],
    ['Iterations', String(world.config.solverIterations)],
  ]
  return (
    <>
      <p className="text-[11px] text-slate-500 leading-relaxed">
        Select a body to edit it. Drag empty space to pan, scroll to zoom, or use the Grab / Spawn /
        Link tools. Space = play/pause, S = step, R = reset, Del = delete selection.
      </p>
      <Section title="Scene">
        {rows.map(([k, v]) => (
          <div key={k} className="flex justify-between text-xs">
            <span className="text-slate-400">{k}</span>
            <span className="text-slate-200 font-mono">{v}</span>
          </div>
        ))}
      </Section>
    </>
  )
}
