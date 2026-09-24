import { useEffect, useRef } from 'react'
import { EnergySample } from '../engine/world'

interface Props {
  history: EnergySample[]
  version: number
}

const WINDOW_S = 8 // seconds of history shown

interface Series {
  ts: number[]
  data: number[]
  color: string
}

function drawPlot(
  ctx: CanvasRenderingContext2D,
  x0: number,
  y0: number,
  w: number,
  h: number,
  series: Series[],
  tNow: number,
): void {
  ctx.fillStyle = 'rgba(15,20,38,0.9)'
  ctx.fillRect(x0, y0, w, h)
  ctx.strokeStyle = 'rgba(120,140,200,0.15)'
  ctx.strokeRect(x0 + 0.5, y0 + 0.5, w - 1, h - 1)

  let min = Infinity
  let max = -Infinity
  for (const s of series) {
    for (const v of s.data) {
      if (v < min) min = v
      if (v > max) max = v
    }
  }
  if (!Number.isFinite(min) || !Number.isFinite(max)) return
  if (max - min < 1e-6) {
    max += 1
    min -= 1
  }

  const pad = 4
  const toY = (v: number) => y0 + h - pad - ((v - min) / (max - min)) * (h - 2 * pad)

  // zero line (if in range)
  if (min < 0 && max > 0) {
    ctx.strokeStyle = 'rgba(120,140,200,0.25)'
    ctx.beginPath()
    ctx.moveTo(x0, toY(0))
    ctx.lineTo(x0 + w, toY(0))
    ctx.stroke()
  }

  for (const s of series) {
    ctx.strokeStyle = s.color
    ctx.lineWidth = 1
    ctx.beginPath()
    let started = false
    for (let i = 0; i < s.data.length; i++) {
      const X = x0 + w - ((tNow - s.ts[i]) / WINDOW_S) * w
      if (X < x0) continue
      const Y = toY(s.data[i])
      if (!started) {
        ctx.moveTo(X, Y)
        started = true
      } else ctx.lineTo(X, Y)
    }
    ctx.stroke()
  }
}

export function Graphs({ history, version }: Props) {
  const ref = useRef<HTMLCanvasElement | null>(null)

  useEffect(() => {
    void version
    const canvas = ref.current
    if (!canvas) return
    const dpr = window.devicePixelRatio || 1
    const W = 264
    const H = 172
    canvas.width = W * dpr
    canvas.height = H * dpr
    canvas.style.width = W + 'px'
    canvas.style.height = H + 'px'
    const ctx = canvas.getContext('2d')
    if (!ctx) return
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0)
    ctx.clearRect(0, 0, W, H)

    const now = history.length ? history[history.length - 1].t : 0
    const cutoff = now - WINDOW_S
    const win = history.filter((s) => s.t >= cutoff)
    const ts = win.map((s) => s.t)

    ctx.font = '10px ui-monospace, monospace'
    ctx.fillStyle = 'rgba(200,210,240,0.7)'
    ctx.fillText('energy   KE · PE · total', 6, 11)
    drawPlot(ctx, 0, 15, W, 70, [
      { ts, data: win.map((s) => s.ke), color: '#4ade80' },
      { ts, data: win.map((s) => s.pe), color: '#60a5fa' },
      { ts, data: win.map((s) => s.total), color: '#e2e8f0' },
    ], now)

    ctx.fillStyle = 'rgba(200,210,240,0.7)'
    ctx.fillText('momentum  px · py', 6, 99)
    drawPlot(ctx, 0, 103, W, 62, [
      { ts, data: win.map((s) => s.px), color: '#fbbf24' },
      { ts, data: win.map((s) => s.py), color: '#f472b6' },
    ], now)
  }, [history, version])

  return (
    <div className="border-t border-slate-700/50 p-3">
      <div className="text-[10px] uppercase tracking-wider text-slate-500 mb-1.5">Live graphs</div>
      <canvas ref={ref} className="block" />
    </div>
  )
}
