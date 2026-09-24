/**
 * Tiny Web Audio impact synth. Each contact impulse triggers a short filtered
 * noise burst whose pitch/volume scale with the impulse. Lazily created on the
 * first user gesture (browsers require an AudioContext to be resumed after a
 * gesture), and capped so a busy frame can't spawn unbounded nodes.
 */
export class ImpactAudio {
  private ctx: AudioContext | null = null
  private master: GainNode | null = null
  private noise: AudioBuffer | null = null
  private last = 0
  enabled = true

  /** call from a click/touch handler to unlock the context. */
  unlock(): void {
    if (!this.ctx) {
      const AC: typeof AudioContext | undefined =
        window.AudioContext ??
        (window as unknown as { webkitAudioContext?: typeof AudioContext }).webkitAudioContext
      if (!AC) return
      this.ctx = new AC()
      this.master = this.ctx.createGain()
      this.master.gain.value = 0.5
      this.master.connect(this.ctx.destination)
      const len = Math.floor(this.ctx.sampleRate * 0.25)
      this.noise = this.ctx.createBuffer(1, len, this.ctx.sampleRate)
      const data = this.noise.getChannelData(0)
      for (let i = 0; i < len; i++) data[i] = Math.random() * 2 - 1
    }
    if (this.ctx.state === 'suspended') void this.ctx.resume()
  }

  /** play a thock for a contact of the given impulse magnitude. */
  impact(impulse: number, x = 0, y = 0): void {
    if (!this.enabled || !this.ctx || !this.master || !this.noise) return
    const now = this.ctx.currentTime
    // rate limit: max ~40 impacts/sec, louder ones get priority
    if (now - this.last < 0.025 && impulse < 8) return
    this.last = now

    const strength = Math.min(1, impulse / 40)
    if (strength < 0.04) return

    const src = this.ctx.createBufferSource()
    src.buffer = this.noise
    // higher impulse -> slightly slower (lower) playback, i.e. a deeper thock
    src.playbackRate.value = 0.6 + Math.random() * 0.5

    const filter = this.ctx.createBiquadFilter()
    filter.type = 'lowpass'
    filter.frequency.value = 1200 + 5000 * strength
    filter.Q.value = 0.8

    const gain = this.ctx.createGain()
    const dur = 0.03 + 0.09 * strength
    gain.gain.setValueAtTime(0.0001, now)
    gain.gain.exponentialRampToValueAtTime(0.05 + 0.6 * strength, now + 0.002)
    gain.gain.exponentialRampToValueAtTime(0.0001, now + dur)

    // light stereo pan from world x
    let pan = 0
    if (this.ctx.createStereoPanner) {
      const p = this.ctx.createStereoPanner()
      pan = Math.max(-1, Math.min(1, x / 6))
      p.pan.value = pan
      src.connect(filter)
      filter.connect(gain)
      gain.connect(p)
      p.connect(this.master)
    } else {
      src.connect(filter)
      filter.connect(gain)
      gain.connect(this.master)
    }
    void y
    src.start(now)
    src.stop(now + dur + 0.02)
  }
}
