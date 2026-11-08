<script setup lang="ts">
// The solutions of a problem, drawn in the browser from the trajectories published by the
// convergence crons (#314; `docs/results.jl` copies them into the site). The reader chooses
// the solves to overlay, the view (against time, phase plane, 3D), the components, and moves
// a time cursor shared by every view, or plays it.
import { computed, onBeforeUnmount, onMounted, ref, shallowRef, watch } from 'vue'

interface Names {
  state: string[]
  control: string[]
  variable: string[]
}
interface Trajectory {
  time: number[]
  state: Record<string, number[]>
  control: Record<string, number[]>
  costate: Record<string, number[]>
  variable: number[]
  names?: Names
}
interface Entry {
  scope: string
  modeling: string
  configuration: string
  platform: string
  fingerprint?: string
  objective: number | null
  result: string
  iterations: number | null
  trajectory: string | null
  run: string
  date: string
}
interface Payload {
  problem: string
  fingerprint: string
  entries: Entry[]
}
type Group = 'time' | 'state' | 'control' | 'costate'
interface Quantity {
  key: string
  group: Group
  name: string
}
interface Series {
  id: string
  entry: Entry
  label: string
  slot: number
  dash: string
  stale: boolean
  width: number
}
type View = 'time' | 'phase' | 'space'

const props = defineProps<{
  problem: string
  modelings: string[]
  titles?: Record<string, string>
}>()

const base = import.meta.env.BASE_URL
const MAX_SERIES = 8 // the categorical palette has eight slots; never cycled
const DASHES = ['', '6 4', '2 3']
const PLAY_SECONDS = 6

// -----------------------------------
// Data
// -----------------------------------
const payload = shallowRef<Payload | null>(null)
const failed = ref(false)
const loaded = shallowRef<Record<string, Trajectory>>({})
const loading = ref(false)

onMounted(async () => {
  try {
    const response = await fetch(`${base}results/${props.problem}.json`)
    if (!response.ok) throw new Error(`HTTP ${response.status}`)
    payload.value = await response.json()
  } catch {
    failed.value = true
  }
})

function configurationLabel(e: Entry): string {
  const parts = e.configuration
    .split(/\s+/)
    .map((p) => p.split('=')[1] ?? p)
    .filter(Boolean)
  const configuration = parts.join(' + ')
  if (props.modelings.length < 2) return configuration
  return `${props.titles?.[e.modeling] ?? e.modeling} · ${configuration}`
}

const allSeries = computed<Series[]>(() => {
  const data = payload.value
  if (!data) return []
  const entries = data.entries.filter(
    (e) => e.trajectory && e.scope === e.modeling && props.modelings.includes(e.modeling),
  )
  entries.sort(
    (a, b) =>
      props.modelings.indexOf(a.modeling) - props.modelings.indexOf(b.modeling) ||
      a.configuration.localeCompare(b.configuration),
  )
  const n = Math.min(entries.length, MAX_SERIES)
  return entries.slice(0, MAX_SERIES).map((e, i) => ({
    id: e.trajectory as string,
    entry: e,
    label: configurationLabel(e),
    slot: i + 1, // follows the solve, never its rank among the shown ones
    dash: DASHES[props.modelings.indexOf(e.modeling) % DASHES.length],
    stale: !!e.fingerprint && e.fingerprint !== data.fingerprint,
    // the first solves drawn wider, under the next: solves that agree stay all visible
    width: 2 + Math.min(1.4, 3 / Math.max(1, n - 1)) * (n - 1 - i),
  }))
})
const hiddenSeries = computed(() => {
  const data = payload.value
  if (!data) return 0
  const n = data.entries.filter(
    (e) => e.trajectory && e.scope === e.modeling && props.modelings.includes(e.modeling),
  ).length
  return Math.max(0, n - MAX_SERIES)
})

const selected = ref<string[]>([])
watch(allSeries, (list) => {
  selected.value = list.map((s) => s.id)
})

async function load(id: string) {
  if (loaded.value[id]) return
  const response = await fetch(`${base}results/${id}`)
  if (!response.ok) throw new Error(`HTTP ${response.status}`)
  const trajectory = (await response.json()) as Trajectory
  loaded.value = { ...loaded.value, [id]: trajectory }
}

watch(selected, async (ids) => {
  loading.value = true
  try {
    await Promise.all(ids.map(load))
  } catch {
    failed.value = true
  } finally {
    loading.value = false
  }
})

function toggleSeries(id: string) {
  const set = new Set(selected.value)
  set.has(id) ? set.delete(id) : set.add(id)
  selected.value = allSeries.value.map((s) => s.id).filter((x) => set.has(x))
}

const active = computed(() =>
  allSeries.value
    .filter((s) => selected.value.includes(s.id) && loaded.value[s.id])
    .map((s) => ({ series: s, data: loaded.value[s.id] })),
)

// -----------------------------------
// Quantities
// -----------------------------------
const natural = (a: string, b: string) => a.localeCompare(b, undefined, { numeric: true })

function namesOf(d: Trajectory): Names {
  return (
    d.names ?? {
      state: Object.keys(d.state).sort(natural),
      control: Object.keys(d.control).sort(natural),
      variable: [],
    }
  )
}

const quantities = computed<Quantity[]>(() => {
  const seen = new Map<string, Quantity>()
  const add = (group: Group, name: string) => {
    const key = `${group}:${name}`
    if (!seen.has(key)) seen.set(key, { key, group, name })
  }
  for (const group of ['state', 'control', 'costate'] as const) {
    for (const { data } of active.value) {
      const names = namesOf(data)
      for (const name of group === 'control' ? names.control : names.state) {
        if (data[group][name]) add(group, name)
      }
    }
  }
  return [...seen.values()]
})
const TIME: Quantity = { key: 'time:t', group: 'time', name: 't' }
const axisChoices = computed(() => [TIME, ...quantities.value])

function byKey(key: string): Quantity {
  return axisChoices.value.find((q) => q.key === key) ?? TIME
}

function label(q: Quantity): string {
  if (q.group === 'time') return 'time'
  if (q.group === 'costate') return `p(${q.name})`
  return q.name
}
const GROUP_TITLES: Record<string, string> = {
  state: 'State',
  control: 'Control',
  costate: 'Costate',
}
const groups = computed(() =>
  (['state', 'control', 'costate'] as const)
    .map((g) => ({ group: g, title: GROUP_TITLES[g], items: quantities.value.filter((q) => q.group === g) }))
    .filter((g) => g.items.length > 0),
)

const shown = ref<string[]>([])
const defaults = () => quantities.value.filter((q) => q.group !== 'costate').map((q) => q.key)
watch(quantities, (list, before) => {
  if (!before || before.length === 0) {
    shown.value = defaults()
    resetAxes()
  } else {
    const keys = new Set(list.map((q) => q.key))
    shown.value = shown.value.filter((k) => keys.has(k))
  }
})

function toggleQuantity(key: string) {
  const set = new Set(shown.value)
  set.has(key) ? set.delete(key) : set.add(key)
  shown.value = quantities.value.map((q) => q.key).filter((k) => set.has(k))
}
function showGroup(group: string) {
  const keys = quantities.value.filter((q) => q.group === group).map((q) => q.key)
  const all = keys.every((k) => shown.value.includes(k))
  const set = new Set(shown.value)
  for (const k of keys) all ? set.delete(k) : set.add(k)
  shown.value = quantities.value.map((q) => q.key).filter((k) => set.has(k))
}
const shownQuantities = computed(() => quantities.value.filter((q) => shown.value.includes(q.key)))

function valuesOf(d: Trajectory, q: Quantity): number[] | undefined {
  if (q.group === 'time') return d.time
  return d[q.group][q.name]
}

// -----------------------------------
// Time cursor
// -----------------------------------
const view = ref<View>('time')
const trace = ref(false)
const showTable = ref(false)

const domain = computed<[number, number]>(() => {
  let lo = Infinity
  let hi = -Infinity
  for (const { data } of active.value) {
    lo = Math.min(lo, data.time[0])
    hi = Math.max(hi, data.time[data.time.length - 1])
  }
  return Number.isFinite(lo) ? [lo, hi] : [0, 1]
})
const cursor = ref(0)
watch(domain, ([, hi]) => {
  cursor.value = hi
})

const playing = ref(false)
let frame = 0
let last = 0
function step(now: number) {
  const [lo, hi] = domain.value
  const dt = last ? (now - last) / 1000 : 0
  last = now
  const next = cursor.value + ((hi - lo) * dt) / PLAY_SECONDS
  if (next >= hi) {
    cursor.value = hi
    playing.value = false
    return
  }
  cursor.value = next
  frame = requestAnimationFrame(step)
}
function play() {
  if (playing.value) {
    pause()
    return
  }
  const [lo, hi] = domain.value
  if (cursor.value >= hi) cursor.value = lo
  playing.value = true
  last = 0
  frame = requestAnimationFrame(step)
}
function pause() {
  playing.value = false
  cancelAnimationFrame(frame)
}
onBeforeUnmount(pause)

// the value at `t`, linear between the nodes; NaN outside the time grid
function at(time: number[], values: number[], t: number): number {
  const n = time.length
  if (n === 0 || t < time[0] - 1e-12 || t > time[n - 1] + 1e-12) return NaN
  let lo = 0
  let hi = n - 1
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1
    if (time[mid] <= t) lo = mid
    else hi = mid
  }
  if (hi === lo || time[hi] === time[lo]) return values[lo]
  const w = (t - time[lo]) / (time[hi] - time[lo])
  return values[lo] + w * (values[hi] - values[lo])
}

// the indices of the nodes drawn: all of them, or those up to the cursor when tracing
function upTo(time: number[]): number {
  if (!trace.value) return time.length
  let n = 0
  while (n < time.length && time[n] <= cursor.value) n++
  return n
}

// -----------------------------------
// Scales and ticks
// -----------------------------------
function niceStep(span: number, count: number): number {
  const raw = span / count
  const p = 10 ** Math.floor(Math.log10(raw))
  const m = raw / p
  return (m < 1.5 ? 1 : m < 3 ? 2 : m < 7 ? 5 : 10) * p
}
function niceDomain(lo: number, hi: number, count = 4): [number, number, number] {
  if (!Number.isFinite(lo) || !Number.isFinite(hi)) return [0, 1, 0.25]
  if (hi - lo < 1e-12 * Math.max(1, Math.abs(lo))) {
    const pad = Math.abs(lo) > 0 ? Math.abs(lo) * 0.1 : 1
    lo -= pad
    hi += pad
  }
  const s = niceStep(hi - lo, count)
  return [Math.floor(lo / s + 1e-9) * s, Math.ceil(hi / s - 1e-9) * s, s]
}
function ticks([lo, hi, s]: [number, number, number]): number[] {
  const out: number[] = []
  for (let v = Math.ceil(lo / s - 1e-9) * s; v <= hi + s * 1e-6; v += s) {
    out.push(Math.abs(v) < s * 1e-9 ? 0 : v)
  }
  return out
}
function fmt(v: number): string {
  if (!Number.isFinite(v)) return '—'
  const a = Math.abs(v)
  if (a < 1e-15) return '0' // a residue of the solver, not a value
  if (a >= 1e4 || a < 1e-3) return v.toExponential(2)
  return String(Number(v.toPrecision(4)))
}
function extent(arrays: (number[] | undefined)[]): [number, number] {
  let lo = Infinity
  let hi = -Infinity
  for (const a of arrays) {
    if (!a) continue
    for (const v of a) {
      if (!Number.isFinite(v)) continue
      if (v < lo) lo = v
      if (v > hi) hi = v
    }
  }
  return [lo, hi]
}
const lin = (d0: number, d1: number, r0: number, r1: number) => (v: number) =>
  r0 + ((v - d0) / (d1 - d0 || 1)) * (r1 - r0)

function path(xs: number[], ys: number[], n: number, sx: (v: number) => number, sy: (v: number) => number, end?: [number, number]): string {
  let out = ''
  let pen = false
  for (let i = 0; i < n; i++) {
    const x = xs[i]
    const y = ys[i]
    if (!Number.isFinite(x) || !Number.isFinite(y)) {
      pen = false
      continue
    }
    out += `${pen ? 'L' : 'M'}${sx(x).toFixed(1)},${sy(y).toFixed(1)}`
    pen = true
  }
  if (end && pen && Number.isFinite(end[0]) && Number.isFinite(end[1])) {
    out += `L${sx(end[0]).toFixed(1)},${sy(end[1]).toFixed(1)}`
  }
  return out
}

// -----------------------------------
// Against time: one small panel per component
// -----------------------------------
const TW = 360
const TH = 210
const TM = { l: 48, r: 12, t: 10, b: 26 }

// the time axis spans the time grids exactly (a free final time is not rounded up)
const timeX = computed<[number, number, number]>(() => {
  const [lo, hi] = domain.value
  return [lo, hi, niceStep(hi - lo || 1, 5)]
})
const timePanels = computed(() =>
  shownQuantities.value.map((q) => {
    const yd = niceDomain(...extent(active.value.map(({ data }) => valuesOf(data, q))))
    const sx = lin(timeX.value[0], timeX.value[1], TM.l, TW - TM.r)
    const sy = lin(yd[0], yd[1], TH - TM.b, TM.t)
    const lines = active.value.map(({ series, data }) => {
      const ys = valuesOf(data, q) ?? []
      const value = at(data.time, ys, cursor.value)
      const n = upTo(data.time)
      return {
        series,
        d: path(data.time, ys, n, sx, sy, trace.value ? [cursor.value, value] : undefined),
        value,
        dot: Number.isFinite(value) ? { x: sx(cursor.value), y: sy(value) } : null,
      }
    })
    return {
      q,
      lines,
      xticks: ticks(timeX.value).map((v) => ({ v, x: sx(v) })),
      yticks: ticks(yd).map((v) => ({ v, y: sy(v) })),
      cursorX: sx(cursor.value),
    }
  }),
)

function svgPoint(event: PointerEvent, width: number, height: number) {
  const rect = (event.currentTarget as SVGElement).getBoundingClientRect()
  return {
    x: ((event.clientX - rect.left) / rect.width) * width,
    y: ((event.clientY - rect.top) / rect.height) * height,
  }
}
function hoverTime(event: PointerEvent) {
  const { x } = svgPoint(event, TW, TH)
  const [d0, d1] = timeX.value
  const t = d0 + ((x - TM.l) / (TW - TM.l - TM.r)) * (d1 - d0)
  const [lo, hi] = domain.value
  pause()
  cursor.value = Math.min(hi, Math.max(lo, t))
}

// -----------------------------------
// Phase plane
// -----------------------------------
// the size of the phase plane and of the 3D view, smaller on a narrow screen so that their
// text keeps its size (the views scale with the width of the page)
const PW = ref(560)
const PH = ref(400)
const PM = { l: 60, r: 16, t: 14, b: 40 }
const axisX = ref(TIME.key)
const axisY = ref(TIME.key)
const axisZ = ref(TIME.key)

function resetAxes() {
  const qs = quantities.value
  const states = qs.filter((q) => q.group === 'state')
  const controls = qs.filter((q) => q.group === 'control')
  const pick = (i: number) => (states[i] ?? controls[i - states.length] ?? TIME).key
  axisX.value = pick(0)
  axisY.value = states.length > 1 ? states[1].key : (controls[0] ?? TIME).key
  axisZ.value = states.length > 2 ? states[2].key : TIME.key
}

const phase = computed(() => {
  const qx = byKey(axisX.value)
  const qy = byKey(axisY.value)
  const xd = niceDomain(...extent(active.value.map(({ data }) => valuesOf(data, qx))))
  const yd = niceDomain(...extent(active.value.map(({ data }) => valuesOf(data, qy))))
  const sx = lin(xd[0], xd[1], PM.l, PW.value - PM.r)
  const sy = lin(yd[0], yd[1], PH.value - PM.b, PM.t)
  const lines = active.value.map(({ series, data }) => {
    const xs = valuesOf(data, qx) ?? []
    const ys = valuesOf(data, qy) ?? []
    const cx = at(data.time, xs, cursor.value)
    const cy = at(data.time, ys, cursor.value)
    const n = upTo(data.time)
    const ok = (i: number) => Number.isFinite(xs[i]) && Number.isFinite(ys[i])
    return {
      series,
      d: path(xs, ys, n, sx, sy, trace.value ? [cx, cy] : undefined),
      start: ok(0) ? { x: sx(xs[0]), y: sy(ys[0]) } : null,
      end: !trace.value && ok(xs.length - 1) ? { x: sx(xs[xs.length - 1]), y: sy(ys[ys.length - 1]) } : null,
      dot: Number.isFinite(cx) && Number.isFinite(cy) ? { x: sx(cx), y: sy(cy) } : null,
    }
  })
  return {
    qx,
    qy,
    lines,
    xticks: ticks(xd).map((v) => ({ v, x: sx(v) })),
    yticks: ticks(yd).map((v) => ({ v, y: sy(v) })),
  }
})

// -----------------------------------
// 3D: orthographic, rotated by dragging
// -----------------------------------
const SW = ref(560)
const SH = ref(440)
const root = ref<HTMLElement | null>(null)
let observer: ResizeObserver | null = null
onMounted(() => {
  observer = new ResizeObserver(([entry]) => {
    const narrow = entry.contentRect.width < 520
    PW.value = SW.value = narrow ? 380 : 560
    PH.value = narrow ? 320 : 400
    SH.value = narrow ? 360 : 440
  })
  if (root.value) observer.observe(root.value)
})
onBeforeUnmount(() => observer?.disconnect())
const DEFAULT_ANGLES = { yaw: -0.65, pitch: 0.45 }
const yaw = ref(DEFAULT_ANGLES.yaw)
const pitch = ref(DEFAULT_ANGLES.pitch)
let drag: { x: number; y: number } | null = null

function project(p: [number, number, number]): { x: number; y: number } {
  const [x, y, z] = p
  const cy = Math.cos(yaw.value)
  const sy = Math.sin(yaw.value)
  const x1 = x * cy - y * sy
  const y1 = x * sy + y * cy
  const up = z * Math.cos(pitch.value) + y1 * Math.sin(pitch.value)
  const r = Math.min(SW.value, SH.value) * 0.3
  return { x: SW.value / 2 + x1 * r, y: SH.value / 2 - up * r }
}
const CORNERS: [number, number, number][] = []
for (const x of [-1, 1]) for (const y of [-1, 1]) for (const z of [-1, 1]) CORNERS.push([x, y, z])
const EDGES = CORNERS.flatMap((a, i) =>
  CORNERS.slice(i + 1)
    .filter((b) => a.filter((v, k) => v !== b[k]).length === 1)
    .map((b) => [a, b] as const),
)

const space = computed(() => {
  const qs = [byKey(axisX.value), byKey(axisY.value), byKey(axisZ.value)]
  const ranges = qs.map((q) => extent(active.value.map(({ data }) => valuesOf(data, q))))
  const norm = ranges.map(([lo, hi]) => {
    const mid = (lo + hi) / 2
    const half = (hi - lo) / 2 || 1
    return (v: number) => (v - mid) / half
  })
  const P = (vals: number[]) => project([norm[0](vals[0]), norm[1](vals[1]), norm[2](vals[2])])
  const lines = active.value.map(({ series, data }) => {
    const arrays = qs.map((q) => valuesOf(data, q) ?? [])
    const n = upTo(data.time)
    let d = ''
    let pen = false
    for (let i = 0; i < n; i++) {
      const v = arrays.map((a) => a[i])
      if (!v.every(Number.isFinite)) {
        pen = false
        continue
      }
      const p = P(v)
      d += `${pen ? 'L' : 'M'}${p.x.toFixed(1)},${p.y.toFixed(1)}`
      pen = true
    }
    const c = arrays.map((a) => at(data.time, a, cursor.value))
    const dot = c.every(Number.isFinite) ? P(c) : null
    if (trace.value && dot && pen) d += `L${dot.x.toFixed(1)},${dot.y.toFixed(1)}`
    const first = arrays.map((a) => a[0])
    return { series, d, dot, start: first.every(Number.isFinite) ? P(first) : null }
  })
  const edges = EDGES.map(([a, b]) => ({ a: project(a), b: project(b) }))
  // each axis along one edge of the box, with its range at the two ends
  const axes = qs.map((q, k) => {
    const from: [number, number, number] = [-1, -1, -1]
    const to: [number, number, number] = [-1, -1, -1]
    to[k] = 1
    const out: [number, number, number] = [-1, -1, -1]
    out[k] = 1.25
    return {
      label: label(q),
      a: project(from),
      b: project(to),
      tip: project(out),
      lo: fmt(ranges[k][0]),
      hi: fmt(ranges[k][1]),
    }
  })
  return { lines, edges, axes }
})

function startDrag(event: PointerEvent) {
  ;(event.currentTarget as Element).setPointerCapture(event.pointerId)
  drag = { x: event.clientX, y: event.clientY }
}
function moveDrag(event: PointerEvent) {
  if (!drag) return
  yaw.value += (event.clientX - drag.x) * 0.01
  pitch.value = Math.max(-1.5, Math.min(1.5, pitch.value + (event.clientY - drag.y) * 0.01))
  drag = { x: event.clientX, y: event.clientY }
}
function endDrag() {
  drag = null
}
function rotateKey(event: KeyboardEvent) {
  const d = 0.1
  if (event.key === 'ArrowLeft') yaw.value -= d
  else if (event.key === 'ArrowRight') yaw.value += d
  else if (event.key === 'ArrowUp') pitch.value = Math.max(-1.5, pitch.value - d)
  else if (event.key === 'ArrowDown') pitch.value = Math.min(1.5, pitch.value + d)
  else return
  event.preventDefault()
}
function resetRotation() {
  yaw.value = DEFAULT_ANGLES.yaw
  pitch.value = DEFAULT_ANGLES.pitch
}

// -----------------------------------
// Readouts
// -----------------------------------
const tableQuantities = computed(() => {
  if (view.value === 'time') return shownQuantities.value
  const keys = view.value === 'phase' ? [axisX.value, axisY.value] : [axisX.value, axisY.value, axisZ.value]
  return [...new Set(keys)].map(byKey).filter((q) => q.group !== 'time')
})
const tableRows = computed(() =>
  tableQuantities.value.map((q) => ({
    q,
    values: active.value.map(({ data }) => fmt(at(data.time, valuesOf(data, q) ?? [], cursor.value))),
  })),
)
function variableText(d: Trajectory): string {
  const names = namesOf(d).variable
  if (!d.variable?.length) return ''
  return d.variable.map((v, i) => `${names[i] ?? `v${i + 1}`} = ${fmt(v)}`).join(', ')
}
const VIEWS: { id: View; label: string }[] = [
  { id: 'time', label: 'Against time' },
  { id: 'phase', label: 'Phase plane' },
  { id: 'space', label: '3D' },
]
</script>

<template>
  <div ref="root" class="problem-plot">
    <p v-if="failed" class="plot-empty">
      No trajectory to draw yet: they come from the convergence crons.
    </p>
    <p v-else-if="!payload" class="plot-empty">Loading the solutions…</p>
    <p v-else-if="allSeries.length === 0" class="plot-empty">
      No trajectory to draw yet for this modeling: they come from its convergence cron.
    </p>
    <template v-else>
      <!-- the solves, as the legend: each one is shown or hidden -->
      <div class="plot-legend" role="group" aria-label="Solves drawn">
        <button
          v-for="s in allSeries"
          :key="s.id"
          type="button"
          class="plot-chip"
          :class="{ off: !selected.includes(s.id) }"
          :aria-pressed="selected.includes(s.id)"
          @click="toggleSeries(s.id)"
        >
          <svg class="plot-key" width="22" height="10" aria-hidden="true">
            <line x1="1" y1="5" x2="21" y2="5" :class="`stroke-${s.slot}`" :stroke-dasharray="s.dash" />
          </svg>
          <span class="plot-chip-label">{{ s.label }}</span>
          <span class="plot-chip-meta">J = {{ fmt(s.entry.objective ?? NaN) }}</span>
          <span v-if="s.stale" class="plot-chip-stale" title="Solved for an earlier version of the problem">older version</span>
        </button>
        <span v-if="hiddenSeries > 0" class="plot-muted">and {{ hiddenSeries }} more, not drawn</span>
      </div>

      <div class="plot-toolbar">
        <div class="plot-views" role="tablist" aria-label="View">
          <button
            v-for="v in VIEWS"
            :key="v.id"
            type="button"
            role="tab"
            :aria-selected="view === v.id"
            :class="{ active: view === v.id }"
            @click="view = v.id"
          >
            {{ v.label }}
          </button>
        </div>
        <div class="plot-time">
          <button type="button" class="plot-play" :aria-label="playing ? 'Pause' : 'Play'" @click="play">
            <svg v-if="!playing" width="12" height="12" viewBox="0 0 12 12" aria-hidden="true"><path d="M2 1 L11 6 L2 11 Z" /></svg>
            <svg v-else width="12" height="12" viewBox="0 0 12 12" aria-hidden="true"><rect x="2" y="1" width="3" height="10" /><rect x="7" y="1" width="3" height="10" /></svg>
          </button>
          <input
            v-model.number="cursor"
            type="range"
            aria-label="Time"
            :min="domain[0]"
            :max="domain[1]"
            :step="(domain[1] - domain[0]) / 1000 || 0.001"
            @input="pause"
          />
          <span class="plot-t">t = {{ fmt(cursor) }}</span>
          <label class="plot-check"><input v-model="trace" type="checkbox" /> Draw up to t</label>
        </div>
      </div>

      <!-- what is drawn -->
      <div v-if="view === 'time'" class="plot-quantities">
        <div v-for="g in groups" :key="g.group" class="plot-group" role="group" :aria-label="g.title">
          <button type="button" class="plot-group-title" @click="showGroup(g.group)">{{ g.title }}</button>
          <button
            v-for="q in g.items"
            :key="q.key"
            type="button"
            class="plot-pill"
            :aria-pressed="shown.includes(q.key)"
            :class="{ on: shown.includes(q.key) }"
            @click="toggleQuantity(q.key)"
          >
            {{ q.name }}
          </button>
        </div>
        <button type="button" class="plot-link" @click="shown = quantities.map((q) => q.key)">All</button>
        <button type="button" class="plot-link" @click="shown = defaults()">Default</button>
      </div>
      <div v-else class="plot-quantities">
        <label class="plot-select">
          x
          <select v-model="axisX">
            <option v-for="q in axisChoices" :key="q.key" :value="q.key">{{ label(q) }}</option>
          </select>
        </label>
        <label class="plot-select">
          y
          <select v-model="axisY">
            <option v-for="q in axisChoices" :key="q.key" :value="q.key">{{ label(q) }}</option>
          </select>
        </label>
        <label v-if="view === 'space'" class="plot-select">
          z
          <select v-model="axisZ">
            <option v-for="q in axisChoices" :key="q.key" :value="q.key">{{ label(q) }}</option>
          </select>
        </label>
        <button v-if="view === 'space'" type="button" class="plot-link" @click="resetRotation">Reset the view</button>
        <span v-if="view === 'space'" class="plot-muted">Drag, or use the arrow keys, to rotate.</span>
      </div>

      <div class="plot-stage" :class="{ refreshing: loading }">
        <!-- against time -->
        <div v-if="view === 'time'" class="plot-grid">
          <p v-if="timePanels.length === 0" class="plot-empty">Choose a component above.</p>
          <figure v-for="panel in timePanels" :key="panel.q.key" class="plot-panel">
            <figcaption>
              <span class="plot-title">{{ label(panel.q) }}</span>
              <span class="plot-group-name">{{ GROUP_TITLES[panel.q.group] }}</span>
              <span class="plot-readout">
                <span v-for="line in panel.lines" :key="line.series.id">
                  <svg class="plot-key" width="14" height="8" aria-hidden="true"><line x1="1" y1="4" x2="13" y2="4" :class="`stroke-${line.series.slot}`" :stroke-dasharray="line.series.dash" /></svg>
                  <strong>{{ fmt(line.value) }}</strong>
                </span>
              </span>
            </figcaption>
            <svg
              :viewBox="`0 0 ${TW} ${TH}`"
              class="plot-svg plot-hover"
              role="img"
              :aria-label="`${label(panel.q)} against time`"
              @pointermove="hoverTime"
              @pointerdown="hoverTime"
            >
              <g class="plot-grid-lines">
                <line v-for="t in panel.yticks" :key="`y${t.v}`" :x1="TM.l" :x2="TW - TM.r" :y1="t.y" :y2="t.y" />
              </g>
              <g class="plot-axis">
                <line :x1="TM.l" :x2="TW - TM.r" :y1="TH - TM.b" :y2="TH - TM.b" />
                <text v-for="t in panel.xticks" :key="`x${t.v}`" :x="t.x" :y="TH - TM.b + 15" text-anchor="middle">{{ fmt(t.v) }}</text>
                <text v-for="t in panel.yticks" :key="`yl${t.v}`" :x="TM.l - 6" :y="t.y + 3.5" text-anchor="end">{{ fmt(t.v) }}</text>
              </g>
              <line class="plot-cursor" :x1="panel.cursorX" :x2="panel.cursorX" :y1="TM.t" :y2="TH - TM.b" />
              <path
                v-for="line in panel.lines"
                :key="line.series.id"
                :d="line.d"
                class="plot-line"
                :class="`stroke-${line.series.slot}`"
                :stroke-dasharray="line.series.dash"
                :stroke-width="line.series.width"
              />
              <template v-for="line in panel.lines" :key="`dot${line.series.id}`">
                <circle v-if="line.dot" :cx="line.dot.x" :cy="line.dot.y" r="4" class="plot-dot" :class="`fill-${line.series.slot}`" />
              </template>
            </svg>
          </figure>
        </div>

        <!-- phase plane -->
        <figure v-else-if="view === 'phase'" class="plot-panel plot-wide">
          <svg :viewBox="`0 0 ${PW} ${PH}`" class="plot-svg" role="img" :aria-label="`${label(phase.qy)} against ${label(phase.qx)}`">
            <g class="plot-grid-lines">
              <line v-for="t in phase.yticks" :key="`y${t.v}`" :x1="PM.l" :x2="PW - PM.r" :y1="t.y" :y2="t.y" />
              <line v-for="t in phase.xticks" :key="`x${t.v}`" :y1="PM.t" :y2="PH - PM.b" :x1="t.x" :x2="t.x" />
            </g>
            <g class="plot-axis">
              <text v-for="t in phase.xticks" :key="`xl${t.v}`" :x="t.x" :y="PH - PM.b + 16" text-anchor="middle">{{ fmt(t.v) }}</text>
              <text v-for="t in phase.yticks" :key="`yl${t.v}`" :x="PM.l - 6" :y="t.y + 3.5" text-anchor="end">{{ fmt(t.v) }}</text>
              <text class="plot-axis-title" :x="(PM.l + PW - PM.r) / 2" :y="PH - 6" text-anchor="middle">{{ label(phase.qx) }}</text>
              <text class="plot-axis-title" :x="14" :y="(PM.t + PH - PM.b) / 2" text-anchor="middle" :transform="`rotate(-90 14 ${(PM.t + PH - PM.b) / 2})`">{{ label(phase.qy) }}</text>
            </g>
            <path v-for="line in phase.lines" :key="line.series.id" :d="line.d" class="plot-line" :class="`stroke-${line.series.slot}`" :stroke-dasharray="line.series.dash" :stroke-width="line.series.width" />
            <template v-for="line in phase.lines" :key="`m${line.series.id}`">
              <circle v-if="line.start" :cx="line.start.x" :cy="line.start.y" r="4.5" class="plot-start" :class="`stroke-${line.series.slot}`" />
              <rect v-if="line.end" :x="line.end.x - 4" :y="line.end.y - 4" width="8" height="8" rx="1.5" class="plot-dot" :class="`fill-${line.series.slot}`" />
              <circle v-if="line.dot" :cx="line.dot.x" :cy="line.dot.y" r="5" class="plot-dot" :class="`fill-${line.series.slot}`" />
            </template>
          </svg>
          <figcaption class="plot-muted">Hollow circle: the initial point; square: the final one; dot: the point at t.</figcaption>
        </figure>

        <!-- 3D -->
        <figure v-else class="plot-panel plot-wide">
          <svg
            :viewBox="`0 0 ${SW} ${SH}`"
            class="plot-svg plot-rotate"
            role="img"
            tabindex="0"
            :aria-label="`${space.axes.map((a) => a.label).join(', ')} in 3D; arrow keys rotate`"
            @pointerdown="startDrag"
            @pointermove="moveDrag"
            @pointerup="endDrag"
            @pointercancel="endDrag"
            @keydown="rotateKey"
          >
            <g class="plot-box">
              <line v-for="(e, i) in space.edges" :key="i" :x1="e.a.x" :y1="e.a.y" :x2="e.b.x" :y2="e.b.y" />
            </g>
            <g class="plot-axis">
              <template v-for="a in space.axes" :key="a.label">
                <line class="plot-axis-edge" :x1="a.a.x" :y1="a.a.y" :x2="a.b.x" :y2="a.b.y" />
                <text class="plot-axis-title" :x="a.tip.x" :y="a.tip.y + 4" text-anchor="middle">{{ a.label }}</text>
                <text :x="a.b.x" :y="a.b.y + 14" text-anchor="middle">{{ a.hi }}</text>
              </template>
            </g>
            <path v-for="line in space.lines" :key="line.series.id" :d="line.d" class="plot-line" :class="`stroke-${line.series.slot}`" :stroke-dasharray="line.series.dash" :stroke-width="line.series.width" />
            <template v-for="line in space.lines" :key="`m${line.series.id}`">
              <circle v-if="line.start" :cx="line.start.x" :cy="line.start.y" r="4.5" class="plot-start" :class="`stroke-${line.series.slot}`" />
              <circle v-if="line.dot" :cx="line.dot.x" :cy="line.dot.y" r="5" class="plot-dot" :class="`fill-${line.series.slot}`" />
            </template>
          </svg>
          <figcaption class="plot-muted">
            Each axis spans the values drawn: {{ space.axes.map((a) => `${a.label} from ${a.lo} to ${a.hi}`).join('; ') }}.
          </figcaption>
        </figure>
      </div>

      <p v-if="active.some(({ data }) => variableText(data))" class="plot-muted plot-variables">
        Variable: {{ active.map(({ series, data }) => `${variableText(data)} (${series.label})`).join('; ') }}
      </p>
      <button type="button" class="plot-link" :aria-expanded="showTable" @click="showTable = !showTable">
        {{ showTable ? 'Hide' : 'Show' }} the values at t = {{ fmt(cursor) }}
      </button>
      <div v-if="showTable" class="plot-table">
        <table>
          <thead>
            <tr>
              <th>Component</th>
              <th v-for="{ series } in active" :key="series.id">{{ series.label }}</th>
            </tr>
          </thead>
          <tbody>
            <tr v-for="row in tableRows" :key="row.q.key">
              <td>{{ label(row.q) }}</td>
              <td v-for="(v, i) in row.values" :key="i">{{ v }}</td>
            </tr>
          </tbody>
        </table>
      </div>
    </template>
  </div>
</template>

<style scoped>
/* The reference categorical palette, eight slots in a fixed order (validated light and dark);
   each solve keeps its slot whatever the others shown. */
.problem-plot {
  --plot-1: #2a78d6;
  --plot-2: #eb6834;
  --plot-3: #1baf7a;
  --plot-4: #eda100;
  --plot-5: #e87ba4;
  --plot-6: #008300;
  --plot-7: #6250d6;
  --plot-8: #e34948;
  --plot-surface: var(--vp-c-bg);
  --plot-grid: var(--vp-c-divider);
  --plot-ink: var(--vp-c-text-2);
  margin: 1rem 0 1.5rem;
}

.stroke-1 { stroke: var(--plot-1); }
.stroke-2 { stroke: var(--plot-2); }
.stroke-3 { stroke: var(--plot-3); }
.stroke-4 { stroke: var(--plot-4); }
.stroke-5 { stroke: var(--plot-5); }
.stroke-6 { stroke: var(--plot-6); }
.stroke-7 { stroke: var(--plot-7); }
.stroke-8 { stroke: var(--plot-8); }
.fill-1 { fill: var(--plot-1); }
.fill-2 { fill: var(--plot-2); }
.fill-3 { fill: var(--plot-3); }
.fill-4 { fill: var(--plot-4); }
.fill-5 { fill: var(--plot-5); }
.fill-6 { fill: var(--plot-6); }
.fill-7 { fill: var(--plot-7); }
.fill-8 { fill: var(--plot-8); }

.plot-empty,
.plot-muted {
  color: var(--vp-c-text-2);
  font-size: 0.85rem;
}
.plot-empty {
  padding: 1rem;
  border: 1px dashed var(--vp-c-divider);
  border-radius: 8px;
  text-align: center;
}

.plot-legend,
.plot-toolbar,
.plot-quantities {
  display: flex;
  flex-wrap: wrap;
  align-items: center;
  gap: 0.4rem 0.6rem;
  margin: 0 0 0.6rem;
}
.plot-toolbar {
  justify-content: space-between;
}

button {
  font: inherit;
  color: inherit;
  cursor: pointer;
}
.plot-chip {
  display: inline-flex;
  align-items: center;
  gap: 0.4rem;
  padding: 0.2rem 0.7rem;
  border: 1px solid var(--vp-c-divider);
  border-radius: 1rem;
  background: var(--vp-c-bg-soft);
  font-size: 0.82rem;
  color: var(--vp-c-text-1);
}
.plot-chip.off {
  opacity: 0.5;
  background: transparent;
}
.plot-chip-meta {
  color: var(--vp-c-text-2);
  font-variant-numeric: tabular-nums;
}
.plot-chip-stale {
  padding: 0 0.4rem;
  border-radius: 0.6rem;
  background: var(--vp-c-warning-soft);
  color: var(--vp-c-warning-1);
  font-size: 0.75rem;
}
.plot-key line {
  stroke-width: 2.5;
  stroke-linecap: round;
}

.plot-views {
  display: inline-flex;
  border: 1px solid var(--vp-c-divider);
  border-radius: 8px;
  overflow: hidden;
}
.plot-views button {
  padding: 0.3rem 0.8rem;
  border: none;
  background: transparent;
  font-size: 0.85rem;
  color: var(--vp-c-text-2);
}
.plot-views button + button {
  border-left: 1px solid var(--vp-c-divider);
}
.plot-views button.active {
  background: var(--vp-c-brand-soft);
  color: var(--vp-c-brand-1);
  font-weight: 600;
}

.plot-time {
  display: flex;
  flex-wrap: wrap;
  flex: 1 1 16rem;
  align-items: center;
  gap: 0.4rem 0.5rem;
  min-width: 0;
  max-width: 32rem;
}
.plot-time input[type='range'] {
  flex: 1;
  min-width: 6rem;
  accent-color: var(--vp-c-brand-1);
}
.plot-play {
  display: inline-flex;
  align-items: center;
  justify-content: center;
  width: 2rem;
  height: 2rem;
  border: 1px solid var(--vp-c-divider);
  border-radius: 50%;
  background: var(--vp-c-bg-soft);
}
.plot-play svg {
  fill: var(--vp-c-text-1);
}
.plot-t {
  min-width: 4.5rem;
  font-size: 0.82rem;
  font-variant-numeric: tabular-nums;
  color: var(--vp-c-text-2);
}
.plot-check,
.plot-select {
  display: inline-flex;
  align-items: center;
  gap: 0.3rem;
  font-size: 0.82rem;
  color: var(--vp-c-text-2);
  white-space: nowrap;
}
.plot-select select {
  padding: 0.15rem 0.4rem;
  border: 1px solid var(--vp-c-divider);
  border-radius: 6px;
  background: var(--vp-c-bg);
  color: var(--vp-c-text-1);
  font: inherit;
}

.plot-group {
  display: inline-flex;
  flex-wrap: wrap;
  align-items: center;
  gap: 0.25rem;
  margin-right: 0.4rem;
}
.plot-group-title {
  padding: 0 0.2rem;
  border: none;
  background: none;
  font-size: 0.78rem;
  font-weight: 600;
  text-transform: uppercase;
  letter-spacing: 0.03em;
  color: var(--vp-c-text-2);
}
.plot-pill {
  padding: 0.1rem 0.55rem;
  border: 1px solid var(--vp-c-divider);
  border-radius: 1rem;
  background: transparent;
  font-family: var(--vp-font-family-mono);
  font-size: 0.78rem;
  color: var(--vp-c-text-2);
}
.plot-pill.on {
  border-color: var(--vp-c-brand-1);
  background: var(--vp-c-brand-soft);
  color: var(--vp-c-brand-1);
}
.plot-link {
  padding: 0;
  border: none;
  background: none;
  font-size: 0.82rem;
  color: var(--vp-c-brand-1);
}
.plot-link:hover {
  text-decoration: underline;
}

.plot-stage {
  transition: opacity 0.2s;
}
.plot-stage.refreshing {
  opacity: 0.6;
}
.plot-grid {
  display: grid;
  grid-template-columns: repeat(auto-fill, minmax(min(100%, 300px), 1fr));
  gap: 0.75rem;
}
.plot-panel {
  margin: 0;
  padding: 0.5rem 0.6rem 0.3rem;
  border: 1px solid var(--vp-c-divider);
  border-radius: 10px;
  background: var(--plot-surface);
}
.plot-wide {
  max-width: 44rem;
}
.plot-panel figcaption {
  display: flex;
  flex-wrap: wrap;
  align-items: baseline;
  gap: 0.2rem 0.5rem;
  font-size: 0.82rem;
}
.plot-title {
  font-family: var(--vp-font-family-mono);
  font-weight: 600;
  color: var(--vp-c-text-1);
}
.plot-group-name {
  color: var(--vp-c-text-3);
  font-size: 0.75rem;
}
.plot-readout {
  display: inline-flex;
  flex-wrap: wrap;
  gap: 0.2rem 0.6rem;
  margin-left: auto;
  font-variant-numeric: tabular-nums;
  color: var(--vp-c-text-1);
}
.plot-readout > span {
  display: inline-flex;
  align-items: center;
  gap: 0.25rem;
}
.plot-readout strong {
  font-weight: 600;
}

.plot-svg {
  display: block;
  width: 100%;
  height: auto;
  overflow: visible;
}
.plot-hover {
  cursor: crosshair;
  touch-action: pan-y;
}
.plot-rotate {
  cursor: grab;
  touch-action: none;
}
.plot-rotate:active {
  cursor: grabbing;
}
.plot-rotate:focus-visible {
  outline: 2px solid var(--vp-c-brand-1);
  outline-offset: 2px;
  border-radius: 6px;
}
.plot-grid-lines line,
.plot-box line {
  stroke: var(--plot-grid);
  stroke-width: 1;
}
.plot-axis line {
  stroke: var(--vp-c-text-3);
  stroke-width: 1;
}
.plot-axis .plot-axis-edge {
  stroke-width: 1.5;
}
.plot-axis text {
  fill: var(--plot-ink);
  font-size: 11px;
  font-variant-numeric: tabular-nums;
}
.plot-axis .plot-axis-title {
  fill: var(--vp-c-text-1);
  font-family: var(--vp-font-family-mono);
  font-size: 12px;
  font-weight: 600;
}
.plot-cursor {
  stroke: var(--vp-c-text-3);
  stroke-width: 1;
  stroke-dasharray: 3 3;
}
.plot-line {
  fill: none;
  stroke-linejoin: round;
  stroke-linecap: round;
}
.plot-dot {
  stroke: var(--plot-surface);
  stroke-width: 2;
}
.plot-start {
  fill: var(--plot-surface);
  stroke-width: 2;
}
.plot-variables {
  margin: 0.5rem 0 0;
}

.plot-table {
  overflow-x: auto;
  margin-top: 0.4rem;
}
.plot-table table {
  margin: 0;
  font-size: 0.82rem;
  font-variant-numeric: tabular-nums;
}
</style>

<style>
/* the dark steps of the same palette (not scoped: `html.dark` is outside the component) */
html.dark .problem-plot {
  --plot-1: #3987e5;
  --plot-2: #d95926;
  --plot-3: #199e70;
  --plot-4: #c98500;
  --plot-5: #d55181;
  --plot-6: #008300;
  --plot-7: #9085e9;
  --plot-8: #e66767;
}
</style>
