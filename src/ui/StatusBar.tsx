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

interface Props {
  hud: Hud
  presetName: string
}

function Item({ label, value }: { label: string; value: string }) {
  return (
    <span className="flex items-center gap-1 text-[11px]">
      <span className="text-slate-500">{label}</span>
      <span className="font-mono text-slate-300">{value}</span>
    </span>
  )
}

export function StatusBar({ hud, presetName }: Props) {
  return (
    <div className="h-7 shrink-0 border-t border-slate-700/60 bg-slate-900/80 flex items-center gap-4 px-3 overflow-x-auto">
      <span className="text-[11px] text-sky-300 font-medium shrink-0">{presetName}</span>
      <Item label="t" value={hud.t.toFixed(1) + 's'} />
      <Item label="bodies" value={String(hud.bodies)} />
      <Item label="fps" value={hud.fps.toFixed(0)} />
      <Item label="steps/tick" value={String(hud.steps)} />
      <Item label="KE" value={hud.ke.toFixed(2)} />
      <Item label="PE" value={hud.pe.toFixed(2)} />
      <Item label="E" value={hud.total.toFixed(2)} />
      <Item label="px" value={hud.px.toFixed(2)} />
      <Item label="py" value={hud.py.toFixed(2)} />
      <Item label="L" value={hud.L.toFixed(2)} />
    </div>
  )
}
