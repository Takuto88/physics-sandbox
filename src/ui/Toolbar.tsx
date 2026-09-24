import { PRESETS } from '../engine/presets'
import type { Integrator } from '../engine/world'
import { RenderOptions } from './renderer'

type Tool = 'select' | 'grab' | 'spawn' | 'constraint'

interface Props {
  presets: typeof PRESETS
  presetId: string
  onPreset: (id: string) => void
  running: boolean
  onToggleRunning: () => void
  onStep: () => void
  onReset: () => void
  timeScale: number
  onTimeScale: (n: number) => void
  integrator: Integrator
  onIntegrator: (i: Integrator) => void
  gravityMode: 'uniform' | 'nbody' | 'none'
  onGravityMode: (m: 'uniform' | 'nbody' | 'none') => void
  tool: Tool
  onTool: (t: Tool) => void
  spawnShape: 'circle' | 'box'
  onSpawnShape: (s: 'circle' | 'box') => void
  renderOpts: RenderOptions
  onRenderOpts: (o: RenderOptions) => void
  audioOn: boolean
  onAudioToggle: () => void
}

function Btn({
  active,
  onClick,
  title,
  children,
}: {
  active?: boolean
  onClick: () => void
  title?: string
  children: React.ReactNode
}) {
  return (
    <button
      onClick={onClick}
      title={title}
      className={
        'px-2.5 h-7 rounded text-xs font-medium transition-colors border ' +
        (active
          ? 'bg-sky-500/25 border-sky-400/60 text-sky-200'
          : 'bg-slate-800/70 border-slate-700 text-slate-300 hover:bg-slate-700/70')
      }
    >
      {children}
    </button>
  )
}

function Seg<T extends string>({
  value,
  options,
  onChange,
}: {
  value: T
  options: { v: T; label: string; title?: string }[]
  onChange: (v: T) => void
}) {
  return (
    <div className="flex rounded border border-slate-700 overflow-hidden">
      {options.map((o) => (
        <button
          key={o.v}
          title={o.title}
          onClick={() => onChange(o.v)}
          className={
            'px-2 h-7 text-xs font-medium transition-colors ' +
            (value === o.v ? 'bg-sky-500/25 text-sky-200' : 'bg-slate-800/70 text-slate-400 hover:bg-slate-700/70')
          }
        >
          {o.label}
        </button>
      ))}
    </div>
  )
}

const TOOLS: { v: Tool; label: string; title: string }[] = [
  { v: 'select', label: 'Select', title: 'Select / pan (click a body, drag empty space)' },
  { v: 'grab', label: 'Grab', title: 'Grab & throw a body (drag it around)' },
  { v: 'spawn', label: 'Spawn', title: 'Click to drop a new body' },
  { v: 'constraint', label: 'Link', title: 'Click two bodies to connect them with a rod' },
]

export function Toolbar(p: Props) {
  return (
    <div className="flex items-center gap-3 px-3 h-12 border-b border-slate-700/60 bg-slate-900/80 flex-wrap">
      <div className="flex items-center gap-2">
        <span className="text-sm font-semibold text-sky-300 tracking-tight">Physics Sandbox</span>
      </div>

      <select
        value={p.presetId}
        onChange={(e) => p.onPreset(e.target.value)}
        className="h-7 rounded bg-slate-800/70 border border-slate-700 text-xs text-slate-200 px-2 max-w-44"
        title="Load a preset scene"
      >
        {p.presets.map((pr) => (
          <option key={pr.id} value={pr.id}>
            {pr.name}
          </option>
        ))}
      </select>

      <div className="flex items-center gap-1">
        <Btn active={p.running} onClick={p.onToggleRunning} title="Play / pause (Space)">
          {p.running ? 'Pause' : 'Play'}
        </Btn>
        <Btn onClick={p.onStep} title="Advance one 120 Hz step (S)">
          Step
        </Btn>
        <Btn onClick={p.onReset} title="Reload the current preset (R)">
          Reset
        </Btn>
      </div>

      <div className="flex items-center gap-2" title="Simulation speed">
        <span className="text-[11px] text-slate-400 w-8">{p.timeScale.toFixed(1)}×</span>
        <input
          type="range"
          min={0}
          max={4}
          step={0.1}
          value={p.timeScale}
          onChange={(e) => p.onTimeScale(Number(e.target.value))}
          className="w-24 accent-sky-400"
        />
      </div>

      <Seg
        value={p.integrator}
        onChange={p.onIntegrator}
        options={[
          { v: 'verlet', label: 'Verlet', title: '2nd-order symplectic velocity Verlet' },
          { v: 'euler', label: 'Euler', title: 'Semi-implicit (symplectic) Euler' },
        ]}
      />

      <Seg
        value={p.gravityMode}
        onChange={p.onGravityMode}
        options={[
          { v: 'uniform', label: 'Uniform', title: 'Constant gravity field' },
          { v: 'nbody', label: 'N-body', title: 'Mutual point-mass gravity' },
          { v: 'none', label: 'None', title: 'Zero gravity' },
        ]}
      />

      <div className="flex items-center gap-1">
        {TOOLS.map((t) => (
          <Btn key={t.v} active={p.tool === t.v} onClick={() => p.onTool(t.v)} title={t.title}>
            {t.label}
          </Btn>
        ))}
        {p.tool === 'spawn' && (
          <Seg
            value={p.spawnShape}
            onChange={p.onSpawnShape}
            options={[
              { v: 'circle', label: 'Ball' },
              { v: 'box', label: 'Box' },
            ]}
          />
        )}
      </div>

      <div className="flex items-center gap-1 ml-auto">
        <Btn active={p.renderOpts.showGrid} onClick={() => p.onRenderOpts({ ...p.renderOpts, showGrid: !p.renderOpts.showGrid })} title="Show grid">
          Grid
        </Btn>
        <Btn
          active={p.renderOpts.showVelocity}
          onClick={() => p.onRenderOpts({ ...p.renderOpts, showVelocity: !p.renderOpts.showVelocity })}
          title="Show velocity vectors"
        >
          Vec
        </Btn>
        <Btn
          active={p.renderOpts.showLabels}
          onClick={() => p.onRenderOpts({ ...p.renderOpts, showLabels: !p.renderOpts.showLabels })}
          title="Show body labels"
        >
          Labels
        </Btn>
        <Btn active={p.audioOn} onClick={p.onAudioToggle} title="Impact sounds">
          Sound
        </Btn>
      </div>
    </div>
  )
}
