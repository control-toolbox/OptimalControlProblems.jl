<script setup lang="ts">
// The problems as cards (#314): a thumbnail of the solution drawn from the trajectories the
// crons publish, the name, the features, and the status in each modeling. Filters in one row
// above. Data: `problems.json` (docs/browser.jl, the features and statuses) and
// `results/thumbnails.json` (docs/results.jl), both written by the build.
import { computed, onMounted, ref, watch } from 'vue'
import ProblemsBrowser from './ProblemsBrowser.vue'

interface Modeling {
  name: string
  title: string
  status: string
}
interface Row {
  Problem: string
  State: number
  Control: number
  Variable: number
  Cost: string
  FinalTime: string
  TotalConstraints: number
  Modelings: Modeling[]
}
interface Thumbnail {
  time: number[]
  series: { name: string; values: number[] }[]
}

const base = import.meta.env.BASE_URL
const rows = ref<Row[]>([])
const thumbnails = ref<Record<string, Thumbnail>>({})
const error = ref('')
const query = ref('')
const modeling = ref('')
const status = ref('')

// Cards or the table of the features: remembered in the browser, a per-viewer convenience
const STORAGE_KEY = 'ocp.problems.layout'
const layout = ref<'cards' | 'table'>('cards')
watch(layout, (value) => {
  try {
    window.localStorage.setItem(STORAGE_KEY, value)
  } catch {
    // private window or blocked storage: nothing to remember
  }
})

onMounted(async () => {
  try {
    if (window.localStorage.getItem(STORAGE_KEY) === 'table') layout.value = 'table'
  } catch {
    // the default layout
  }
  try {
    const response = await fetch(`${base}problems.json`)
    if (!response.ok) throw new Error(`HTTP ${response.status}`)
    const data = await response.json()
    rows.value = data.Problem.map((p: string, i: number) => ({
      Problem: p,
      State: data.State[i],
      Control: data.Control[i],
      Variable: data.Variable[i],
      Cost: data.Cost[i],
      FinalTime: data.FinalTime[i],
      TotalConstraints: data.TotalConstraints[i],
      Modelings: data.Modelings[i],
    }))
  } catch (e) {
    error.value = String(e)
  }
  try {
    const response = await fetch(`${base}results/thumbnails.json`)
    if (response.ok) thumbnails.value = await response.json()
  } catch {
    // no results yet: the cards show the features only
  }
})

const modelingChoices = computed(() => {
  const seen = new Map<string, string>()
  for (const r of rows.value) for (const m of r.Modelings) seen.set(m.name, m.title)
  return [...seen.entries()].map(([name, title]) => ({ name, title }))
})

const shown = computed(() => {
  const q = query.value.trim().toLowerCase().replaceAll(' ', '_')
  return rows.value.filter((r) => {
    if (q && !r.Problem.toLowerCase().includes(q)) return false
    const ms = modeling.value ? r.Modelings.filter((m) => m.name === modeling.value) : r.Modelings
    if (ms.length === 0) return false
    if (status.value && !ms.some((m) => m.status === status.value)) return false
    return true
  })
})

const title = (p: string) => p.charAt(0).toUpperCase() + p.slice(1).replaceAll('_', ' ')
const url = (p: string) => `${base}problems/${p}`

// each component scaled to the height of the card: the shape of the solution, not its values
const TW = 240
const TH = 96
function paths(t: Thumbnail): { d: string; slot: number }[] {
  const t0 = t.time[0]
  const t1 = t.time[t.time.length - 1]
  const sx = (v: number) => 6 + ((v - t0) / (t1 - t0 || 1)) * (TW - 12)
  return t.series.map((s, k) => {
    const finite = s.values.filter(Number.isFinite)
    const lo = Math.min(...finite)
    const hi = Math.max(...finite)
    const sy = (v: number) => TH - 8 - ((v - lo) / (hi - lo || 1)) * (TH - 16)
    let d = ''
    s.values.forEach((v, i) => {
      if (Number.isFinite(v)) d += `${d ? 'L' : 'M'}${sx(t.time[i]).toFixed(1)},${sy(v).toFixed(1)}`
    })
    return { d, slot: k + 1 }
  })
}
</script>

<template>
  <div class="gallery">
    <div class="gallery-layout" role="tablist" aria-label="Layout">
      <button type="button" role="tab" :aria-selected="layout === 'cards'" :class="{ active: layout === 'cards' }" @click="layout = 'cards'">Cards</button>
      <button type="button" role="tab" :aria-selected="layout === 'table'" :class="{ active: layout === 'table' }" @click="layout = 'table'">Table</button>
    </div>
    <ProblemsBrowser v-if="layout === 'table'" />
    <template v-else>
    <div class="gallery-filters">
      <input v-model="query" type="search" placeholder="Search a problem" aria-label="Search a problem" />
      <label>
        Modeling
        <select v-model="modeling">
          <option value="">all</option>
          <option v-for="m in modelingChoices" :key="m.name" :value="m.name">{{ m.title }}</option>
        </select>
      </label>
      <label>
        Status
        <select v-model="status">
          <option value="">all</option>
          <option value="valid">valid</option>
          <option value="draft">draft</option>
        </select>
      </label>
      <span class="gallery-count">{{ shown.length }} of {{ rows.length }} problems</span>
    </div>

    <p v-if="error" class="gallery-empty">The problems could not be loaded ({{ error }}).</p>
    <div v-else class="gallery-grid">
      <a v-for="r in shown" :key="r.Problem" :href="url(r.Problem)" class="gallery-card">
        <svg v-if="thumbnails[r.Problem]" :viewBox="`0 0 ${TW} ${TH}`" class="gallery-thumb" role="img" :aria-label="`The solution of ${title(r.Problem)}, state against time`">
          <path v-for="p in paths(thumbnails[r.Problem])" :key="p.slot" :d="p.d" :class="`stroke-${p.slot}`" />
        </svg>
        <div v-else class="gallery-thumb gallery-nothumb">no solution published yet</div>
        <div class="gallery-body">
          <h3>{{ title(r.Problem) }}</h3>
          <p class="gallery-facts">
            {{ r.State }} state · {{ r.Control }} control<template v-if="r.Variable"> · {{ r.Variable }} variable</template>
            · {{ r.Cost.toLowerCase() }} cost · {{ r.FinalTime }} final time
          </p>
          <ul class="gallery-modelings">
            <li v-for="m in r.Modelings" :key="m.name" :class="`status-${m.status}`" :title="`${m.title}: ${m.status}`">
              {{ m.title }}
            </li>
          </ul>
        </div>
      </a>
    </div>
    </template>
  </div>
</template>

<style scoped>
.gallery {
  --plot-1: #2a78d6;
  --plot-2: #eb6834;
  --plot-3: #1baf7a;
  margin: 1rem 0;
}
.stroke-1 { stroke: var(--plot-1); }
.stroke-2 { stroke: var(--plot-2); }
.stroke-3 { stroke: var(--plot-3); }

.gallery-layout {
  display: inline-flex;
  margin-bottom: 1rem;
  border: 1px solid var(--vp-c-divider);
  border-radius: 8px;
  overflow: hidden;
}
.gallery-layout button {
  padding: 0.3rem 0.9rem;
  border: none;
  background: transparent;
  font: inherit;
  font-size: 0.85rem;
  color: var(--vp-c-text-2);
  cursor: pointer;
}
.gallery-layout button + button {
  border-left: 1px solid var(--vp-c-divider);
}
.gallery-layout button.active {
  background: var(--vp-c-brand-soft);
  color: var(--vp-c-brand-1);
  font-weight: 600;
}
.gallery-filters {
  display: flex;
  flex-wrap: wrap;
  align-items: center;
  gap: 0.5rem 0.9rem;
  margin-bottom: 1rem;
  font-size: 0.85rem;
  color: var(--vp-c-text-2);
}
.gallery-filters input,
.gallery-filters select {
  padding: 0.3rem 0.6rem;
  border: 1px solid var(--vp-c-divider);
  border-radius: 8px;
  background: var(--vp-c-bg);
  color: var(--vp-c-text-1);
  font: inherit;
}
.gallery-filters input {
  flex: 1 1 12rem;
  max-width: 18rem;
}
.gallery-filters label {
  display: inline-flex;
  align-items: center;
  gap: 0.35rem;
}
.gallery-count {
  margin-left: auto;
  font-variant-numeric: tabular-nums;
}
.gallery-empty {
  color: var(--vp-c-text-2);
}

.gallery-grid {
  display: grid;
  grid-template-columns: repeat(auto-fill, minmax(min(100%, 230px), 1fr));
  gap: 0.9rem;
}
.gallery-card {
  display: flex;
  flex-direction: column;
  overflow: hidden;
  border: 1px solid var(--vp-c-divider);
  border-radius: 12px;
  background: var(--vp-c-bg-soft);
  color: inherit;
  text-decoration: none !important;
  transition: border-color 0.2s, transform 0.2s;
}
.gallery-card:hover,
.gallery-card:focus-visible {
  border-color: var(--vp-c-brand-1);
  transform: translateY(-2px);
}
.gallery-thumb {
  display: block;
  width: 100%;
  height: auto;
  aspect-ratio: 240 / 96;
  background: var(--vp-c-bg);
  border-bottom: 1px solid var(--vp-c-divider);
}
.gallery-thumb path {
  fill: none;
  stroke-width: 2;
  stroke-linejoin: round;
  stroke-linecap: round;
}
.gallery-nothumb {
  display: flex;
  align-items: center;
  justify-content: center;
  color: var(--vp-c-text-3);
  font-size: 0.78rem;
  font-style: italic;
}
.gallery-body {
  padding: 0.6rem 0.8rem 0.7rem;
}
.gallery-body h3 {
  margin: 0 0 0.25rem;
  padding: 0;
  border: none;
  font-size: 1rem;
  line-height: 1.3;
}
.gallery-facts {
  margin: 0 0 0.5rem;
  font-size: 0.78rem;
  line-height: 1.4;
  color: var(--vp-c-text-2);
}
.gallery-modelings {
  display: flex;
  flex-wrap: wrap;
  gap: 0.3rem;
  margin: 0;
  padding: 0;
  list-style: none;
}
.gallery-modelings li {
  margin: 0;
  padding: 0 0.45rem;
  border-radius: 0.6rem;
  font-size: 0.72rem;
  font-weight: 600;
  line-height: 1.6;
}
.gallery-modelings .status-valid {
  background: var(--vp-c-success-soft);
  color: var(--vp-c-success-1);
}
.gallery-modelings .status-draft {
  background: var(--vp-c-warning-soft);
  color: var(--vp-c-warning-1);
}
</style>

<style>
html.dark .gallery {
  --plot-1: #3987e5;
  --plot-2: #d95926;
  --plot-3: #199e70;
}
</style>
