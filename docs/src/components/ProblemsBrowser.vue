<script setup lang="ts">
import { computed, onMounted, ref } from 'vue'

type ConstraintKey = 'x' | 'u' | 'v' | 'c' | 'b'
type SortKey = 'Problem' | 'State' | 'Control' | 'Variable' | 'Cost' | 'FinalTime' | 'TotalConstraints'

interface ProblemRow {
  Problem: string
  State: number
  Control: number
  Variable: number
  Cost: string
  FinalTime: string
  DimStateConstraint: number
  DimControlConstraint: number
  DimVariableConstraint: number
  DimPathConstraint: number
  DimBoundaryConstraint: number
  TotalConstraints: number
}

const constraintColumns: { key: ConstraintKey; label: string; field: keyof ProblemRow }[] = [
  { key: 'x', label: 'State', field: 'DimStateConstraint' },
  { key: 'u', label: 'Control', field: 'DimControlConstraint' },
  { key: 'v', label: 'Variable', field: 'DimVariableConstraint' },
  { key: 'c', label: 'Path', field: 'DimPathConstraint' },
  { key: 'b', label: 'Boundary', field: 'DimBoundaryConstraint' },
]

const rows = ref<ProblemRow[]>([])
const loading = ref(true)
const error = ref('')
const query = ref('')
const cost = ref('')
const finalTime = ref('')
const constraintMode = ref<'any' | 'all'>('any')
const selectedConstraints = ref<ConstraintKey[]>([])
const expandedProblem = ref('')
const sortKey = ref<SortKey>('Problem')
const sortAscending = ref(true)

const baseUrl = import.meta.env.BASE_URL

onMounted(async () => {
  try {
    const response = await fetch(`${baseUrl}problems.json`)
    if (!response.ok) throw new Error(`HTTP ${response.status}`)
    const data = await response.json()
    rows.value = data.Problem.map((problem: string, index: number) => ({
      Problem: problem,
      State: data.State[index],
      Control: data.Control[index],
      Variable: data.Variable[index],
      Cost: data.Cost[index],
      FinalTime: data.FinalTime[index],
      DimStateConstraint: data.DimStateConstraint[index],
      DimControlConstraint: data.DimControlConstraint[index],
      DimVariableConstraint: data.DimVariableConstraint[index],
      DimPathConstraint: data.DimPathConstraint[index],
      DimBoundaryConstraint: data.DimBoundaryConstraint[index],
      TotalConstraints: data.TotalConstraints[index],
    }))
  } catch (caught) {
    error.value = `The browser data could not be loaded (${String(caught)}).`
  } finally {
    loading.value = false
  }
})

const availableCosts = computed(() => [...new Set(rows.value.map((row) => row.Cost))].sort())
const availableFinalTimes = computed(() => [...new Set(rows.value.map((row) => row.FinalTime))].sort())

function matchesQuery(row: ProblemRow): boolean {
  if (!query.value) return true
  return [row.Problem, row.Cost, row.FinalTime].some((value) =>
    value.toLowerCase().includes(query.value.toLowerCase()),
  )
}

function hasConstraint(row: ProblemRow, key: ConstraintKey): boolean {
  const column = constraintColumns.find((item) => item.key === key)
  return column ? Number(row[column.field]) > 0 : false
}

function matchesConstraints(row: ProblemRow): boolean {
  if (!selectedConstraints.value.length) return true
  const matches = selectedConstraints.value.map((key) => hasConstraint(row, key))
  return constraintMode.value === 'all' ? matches.every(Boolean) : matches.some(Boolean)
}

const filteredRows = computed(() => {
  const result = rows.value.filter((row) =>
    matchesQuery(row) &&
    (!cost.value || row.Cost === cost.value) &&
    (!finalTime.value || row.FinalTime === finalTime.value) &&
    matchesConstraints(row),
  )
  return result.sort((left, right) => {
    const first = left[sortKey.value]
    const second = right[sortKey.value]
    const comparison = typeof first === 'number' && typeof second === 'number'
      ? first - second
      : String(first).localeCompare(String(second))
    return sortAscending.value ? comparison : -comparison
  })
})

function toggleSort(key: SortKey): void {
  if (sortKey.value === key) {
    sortAscending.value = !sortAscending.value
  } else {
    sortKey.value = key
    sortAscending.value = true
  }
}

function toggleConstraint(key: ConstraintKey): void {
  selectedConstraints.value = selectedConstraints.value.includes(key)
    ? selectedConstraints.value.filter((item) => item !== key)
    : [...selectedConstraints.value, key]
}

function clearFilters(): void {
  query.value = ''
  cost.value = ''
  finalTime.value = ''
  selectedConstraints.value = []
  constraintMode.value = 'any'
}

function constraintCount(row: ProblemRow, key: ConstraintKey): number {
  const column = constraintColumns.find((item) => item.key === key)
  return column ? Number(row[column.field]) : 0
}

function problemUrl(problem: string): string {
  return `${baseUrl}problems/${problem}`
}

function csvValue(value: string | number): string {
  return `"${String(value).replaceAll('"', '""')}"`
}

function csvText(): string {
  const header = ['Problem', 'State', 'Control', 'Variable', 'Cost', 'Final time', 'Constraints']
  const lines = filteredRows.value.map((row) => [
    row.Problem,
    row.State,
    row.Control,
    row.Variable,
    row.Cost,
    row.FinalTime,
    row.TotalConstraints,
  ].map(csvValue).join(','))
  return [header.map(csvValue).join(','), ...lines].join('\n')
}

function downloadCsv(): void {
  const blob = new Blob([csvText()], { type: 'text/csv;charset=utf-8' })
  const url = URL.createObjectURL(blob)
  const link = document.createElement('a')
  link.href = url
  link.download = 'optimal-control-problems.csv'
  link.click()
  URL.revokeObjectURL(url)
}

async function copyCsv(): Promise<void> {
  await navigator.clipboard.writeText(csvText())
}
</script>

<template>
  <section class="browser">
    <header class="browser-header">
      <div>
        <p class="eyebrow">A field guide to the collection</p>
        <h2>Find a problem by its shape</h2>
        <p class="lede">
          Compare dimensions, objective types, horizons, and constraints before opening
          a full problem page.
        </p>
      </div>
      <div class="result-count" aria-live="polite">
        <strong>{{ filteredRows.length }}</strong>
        <span>of {{ rows.length }} problems</span>
      </div>
    </header>

    <div class="toolbar">
      <label class="search">
        <span>Search</span>
        <input v-model="query" type="search" placeholder="Try “rocket” or “Bolza”">
      </label>
      <label>
        <span>Cost</span>
        <select v-model="cost">
          <option value="">All costs</option>
          <option v-for="option in availableCosts" :key="option" :value="option">{{ option }}</option>
        </select>
      </label>
      <label>
        <span>Final time</span>
        <select v-model="finalTime">
          <option value="">Any horizon</option>
          <option v-for="option in availableFinalTimes" :key="option" :value="option">{{ option }}</option>
        </select>
      </label>
      <button class="quiet-button" type="button" @click="clearFilters">Clear filters</button>
    </div>

    <div class="constraint-toolbar">
      <span class="filter-label">Has constraints on</span>
      <div class="constraint-chips">
        <button
          v-for="column in constraintColumns"
          :key="column.key"
          :class="{ selected: selectedConstraints.includes(column.key) }"
          type="button"
          @click="toggleConstraint(column.key)"
        >
          {{ column.key }} <span>{{ column.label }}</span>
        </button>
      </div>
      <label class="mode">
        <span>Match</span>
        <select v-model="constraintMode">
          <option value="any">any selected type</option>
          <option value="all">all selected types</option>
        </select>
      </label>
    </div>

    <div class="actions">
      <span v-if="loading">Loading the collection…</span>
      <span v-else-if="error" class="error">{{ error }}</span>
      <span v-else>{{ filteredRows.length ? 'Select a row to inspect its constraints.' : 'No problems match these filters.' }}</span>
      <div>
        <button type="button" @click="copyCsv">Copy CSV</button>
        <button type="button" @click="downloadCsv">Download CSV</button>
      </div>
    </div>

    <div v-if="!loading && !error" class="table-wrap">
      <table>
        <thead>
          <tr>
            <th><button type="button" @click="toggleSort('Problem')">Problem</button></th>
            <th><button type="button" @click="toggleSort('State')">State</button></th>
            <th><button type="button" @click="toggleSort('Control')">Control</button></th>
            <th><button type="button" @click="toggleSort('Variable')">Variable</button></th>
            <th><button type="button" @click="toggleSort('Cost')">Cost</button></th>
            <th><button type="button" @click="toggleSort('FinalTime')">Horizon</button></th>
            <th><button type="button" @click="toggleSort('TotalConstraints')">Constraints</button></th>
          </tr>
        </thead>
        <tbody>
          <template v-for="row in filteredRows" :key="row.Problem">
            <tr :class="{ expanded: expandedProblem === row.Problem }" @click="expandedProblem = expandedProblem === row.Problem ? '' : row.Problem">
              <td class="problem-name">
                <a :href="problemUrl(row.Problem)" @click.stop>{{ row.Problem.replaceAll('_', ' ') }}</a>
              </td>
              <td>{{ row.State }}</td>
              <td>{{ row.Control }}</td>
              <td>{{ row.Variable }}</td>
              <td><span class="tag">{{ row.Cost }}</span></td>
              <td>{{ row.FinalTime }}</td>
              <td>
                <div class="constraint-summary">
                  <span v-for="column in constraintColumns" :key="column.key" :class="{ inactive: !constraintCount(row, column.key) }">
                    {{ column.key }}{{ constraintCount(row, column.key) }}
                  </span>
                  <strong>{{ row.TotalConstraints }}</strong>
                </div>
              </td>
            </tr>
            <tr v-if="expandedProblem === row.Problem" class="detail-row">
              <td colspan="7">
                <div class="detail">
                  <strong>{{ row.Problem.replaceAll('_', ' ') }}</strong>
                  <span v-for="column in constraintColumns" :key="column.key">
                    {{ column.label }} constraints: {{ constraintCount(row, column.key) }}
                  </span>
                  <a :href="problemUrl(row.Problem)">Open problem page →</a>
                </div>
              </td>
            </tr>
          </template>
        </tbody>
      </table>
    </div>
  </section>
</template>

<style scoped>
.browser {
  --browser-ink: #173b3f;
  --browser-muted: #627579;
  --browser-line: #dce6e2;
  --browser-paper: #f5f8f5;
  --browser-accent: #e56b4a;
  --browser-accent-dark: #b74732;
  color: var(--browser-ink);
  margin: 2.5rem 0;
}

.browser-header {
  align-items: end;
  background: linear-gradient(120deg, #e8f1e9, #fff8ed);
  border: 1px solid var(--browser-line);
  border-radius: 1.25rem;
  display: flex;
  gap: 2rem;
  justify-content: space-between;
  padding: 2rem;
}

.eyebrow {
  color: var(--browser-accent-dark);
  font-size: 0.75rem;
  font-weight: 700;
  letter-spacing: 0.12em;
  margin: 0 0 0.5rem;
  text-transform: uppercase;
}

h2 {
  font-family: Georgia, serif;
  font-size: clamp(1.8rem, 4vw, 3rem);
  letter-spacing: -0.04em;
  line-height: 1;
  margin: 0;
}

.lede {
  color: var(--browser-muted);
  margin: 0.85rem 0 0;
  max-width: 38rem;
}

.result-count {
  background: var(--browser-ink);
  border-radius: 0.8rem;
  color: white;
  min-width: 8rem;
  padding: 0.8rem 1rem;
  text-align: right;
}

.result-count strong,
.result-count span {
  display: block;
}

.result-count strong {
  font-size: 1.7rem;
}

.result-count span {
  color: #c8d8d0;
  font-size: 0.8rem;
}

.toolbar,
.constraint-toolbar,
.actions {
  align-items: end;
  display: flex;
  flex-wrap: wrap;
  gap: 0.8rem;
  margin-top: 1rem;
}

label,
.filter-label {
  color: var(--browser-muted);
  display: grid;
  font-size: 0.75rem;
  font-weight: 700;
  gap: 0.35rem;
  letter-spacing: 0.05em;
  text-transform: uppercase;
}

input,
select,
button {
  font: inherit;
}

input,
select {
  background: white;
  border: 1px solid var(--browser-line);
  border-radius: 0.45rem;
  color: var(--browser-ink);
  min-height: 2.4rem;
  padding: 0.45rem 0.7rem;
}

.search {
  flex: 1 1 16rem;
}

.search input {
  width: 100%;
}

button {
  border: 1px solid var(--browser-line);
  border-radius: 0.45rem;
  cursor: pointer;
  padding: 0.45rem 0.7rem;
}

button:hover,
button:focus-visible {
  border-color: var(--browser-accent);
}

.quiet-button,
.actions button {
  background: var(--browser-paper);
  color: var(--browser-ink);
}

.constraint-toolbar {
  align-items: center;
  border-bottom: 1px solid var(--browser-line);
  padding-bottom: 1rem;
}

.filter-label {
  margin-right: 0.25rem;
}

.constraint-chips {
  display: flex;
  flex-wrap: wrap;
  gap: 0.4rem;
}

.constraint-chips button {
  background: var(--browser-paper);
  color: var(--browser-muted);
  font-size: 0.8rem;
}

.constraint-chips button.selected {
  background: var(--browser-ink);
  border-color: var(--browser-ink);
  color: white;
}

.constraint-chips span {
  margin-left: 0.2rem;
}

.mode {
  margin-left: auto;
}

.actions {
  align-items: center;
  color: var(--browser-muted);
  font-size: 0.85rem;
  justify-content: space-between;
}

.actions > div {
  display: flex;
  gap: 0.4rem;
}

.error {
  color: #ad312d;
}

.table-wrap {
  border: 1px solid var(--browser-line);
  border-radius: 0.75rem;
  margin-top: 0.8rem;
  overflow-x: auto;
}

table {
  border-collapse: collapse;
  min-width: 46rem;
  width: 100%;
}

th {
  background: var(--browser-paper);
  color: var(--browser-muted);
  font-size: 0.72rem;
  letter-spacing: 0.06em;
  text-align: left;
  text-transform: uppercase;
}

th button {
  background: transparent;
  border: 0;
  color: inherit;
  font-size: inherit;
  letter-spacing: inherit;
  padding: 0.85rem 0.7rem;
  text-align: left;
  text-transform: inherit;
  width: 100%;
}

td {
  border-top: 1px solid var(--browser-line);
  padding: 0.7rem;
}

tbody tr:not(.detail-row) {
  cursor: pointer;
}

tbody tr:not(.detail-row):hover,
tbody tr.expanded {
  background: #fff8ed;
}

.problem-name a,
.detail a {
  color: var(--browser-accent-dark);
  font-weight: 700;
  text-decoration: none;
}

.problem-name a:hover,
.detail a:hover {
  text-decoration: underline;
}

.tag {
  background: #edf3ed;
  border-radius: 999px;
  font-size: 0.8rem;
  padding: 0.25rem 0.55rem;
}

.constraint-summary {
  align-items: center;
  display: flex;
  flex-wrap: wrap;
  gap: 0.3rem;
}

.constraint-summary span {
  background: #dfeee4;
  border-radius: 999px;
  font-size: 0.72rem;
  min-width: 1.8rem;
  padding: 0.2rem 0.35rem;
  text-align: center;
}

.constraint-summary span.inactive {
  background: var(--browser-paper);
  color: #a4b1ad;
}

.constraint-summary strong {
  margin-left: 0.25rem;
}

.detail-row td {
  background: #fff8ed;
  padding-top: 0;
}

.detail {
  align-items: center;
  display: flex;
  flex-wrap: wrap;
  gap: 0.7rem 1.2rem;
  padding: 0.2rem 0 0.8rem;
}

@media (max-width: 640px) {
  .browser-header {
    align-items: start;
    flex-direction: column;
    padding: 1.3rem;
  }

  .result-count {
    text-align: left;
  }

  .mode {
    margin-left: 0;
  }
}
</style>
