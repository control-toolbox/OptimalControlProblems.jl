<script setup lang="ts">
import { nextTick, onMounted, ref } from 'vue'
import { currentView, rememberedView, rememberView } from './problemView'

// The tab bar of a problem page, sticky under the navigation bar, so that the modeling is
// chosen at the top of the page instead of scrolled to. `views` is written by the page
// generator (`docs/problems.jl`): the problem, one entry per modeling where it exists, and
// the comparison when there are two modelings or more.
interface View {
  id: string
  label: string
  kind: 'problem' | 'modeling' | 'compare'
  status?: string
}

const props = defineProps<{ views: View[] }>()
const bar = ref<HTMLElement | null>(null)

function available(id: string | null): id is string {
  return id !== null && props.views.some((view) => view.id === id)
}

// The URL wins (a shared link), then the view last chosen in this browser, then the first.
onMounted(() => {
  const fromUrl = new URLSearchParams(window.location.search).get('view')
  const remembered = rememberedView()
  currentView.value = available(fromUrl)
    ? fromUrl
    : available(remembered)
      ? remembered
      : props.views[0].id
})

async function select(id: string): Promise<void> {
  if (id === currentView.value) return
  currentView.value = id
  rememberView(id)
  const url = new URL(window.location.href)
  url.searchParams.set('view', id)
  window.history.replaceState(window.history.state, '', url)
  // Once the bar is stuck, the new view starts above the screen: bring its top under the bar.
  await nextTick()
  const el = bar.value
  const panel = document.getElementById(`view-${id}`)
  if (!el || !panel) return
  const offset = parseFloat(getComputedStyle(el).top) || 0
  const stuck = el.getBoundingClientRect().top <= offset + 1
  if (stuck) {
    const top = panel.getBoundingClientRect().top + window.scrollY
    window.scrollTo({ top: top - offset - el.offsetHeight })
  }
}

// Arrow keys move between the tabs, as in any tab list.
function onKey(event: KeyboardEvent, index: number): void {
  const step = event.key === 'ArrowRight' ? 1 : event.key === 'ArrowLeft' ? -1 : 0
  if (step === 0) return
  event.preventDefault()
  const next = props.views[(index + step + props.views.length) % props.views.length]
  select(next.id)
  nextTick(() => document.getElementById(`tab-${next.id}`)?.focus())
}

function isSelected(view: View, index: number): boolean {
  return currentView.value === '' ? index === 0 : currentView.value === view.id
}
</script>

<template>
  <div ref="bar" class="problem-tabs" role="tablist" aria-label="Views of the problem">
    <template v-for="(view, index) in views" :key="view.id">
      <span v-if="view.kind === 'compare'" class="separator" aria-hidden="true"></span>
      <button
        :id="`tab-${view.id}`"
        type="button"
        role="tab"
        :class="['tab', `tab-${view.kind}`]"
        :aria-selected="isSelected(view, index)"
        :aria-controls="`view-${view.id}`"
        :tabindex="isSelected(view, index) ? 0 : -1"
        @click="select(view.id)"
        @keydown="onKey($event, index)"
      >
        <span v-if="view.kind === 'modeling'" class="eyebrow">Modeling</span>
        <span class="label">{{ view.label }}</span>
        <span v-if="view.status === 'draft'" class="draft" title="Draft in this modeling">draft</span>
      </button>
    </template>
  </div>
</template>

<style scoped>
.problem-tabs {
  position: sticky;
  top: var(--vp-nav-height);
  z-index: 20;
  display: flex;
  flex-wrap: wrap;
  align-items: stretch;
  gap: 0.375rem;
  margin: 1.5rem 0 1rem;
  padding: 0.5rem 0;
  background: var(--vp-c-bg);
  border-bottom: 1px solid var(--vp-c-divider);
}

@media (max-width: 959px) {
  /* the local navigation bar of VitePress is stuck at the top on small screens */
  .problem-tabs {
    top: calc(var(--vp-nav-height) + 1px);
  }
}

.tab {
  display: flex;
  flex-direction: column;
  align-items: flex-start;
  justify-content: center;
  min-height: 2.75rem;
  padding: 0.35rem 0.85rem;
  border: 1px solid var(--vp-c-divider);
  border-radius: 0.6rem;
  background: var(--vp-c-bg-soft);
  color: var(--vp-c-text-2);
  font-size: 0.875rem;
  line-height: 1.25;
  text-align: left;
  transition: border-color 0.15s, color 0.15s, background-color 0.15s;
}

.tab:hover {
  border-color: var(--vp-c-brand-2);
  color: var(--vp-c-text-1);
}

.tab:focus-visible {
  outline: 2px solid var(--vp-c-brand-1);
  outline-offset: 2px;
}

.tab[aria-selected='true'] {
  border-color: var(--vp-c-brand-1);
  background: var(--vp-c-brand-soft);
  color: var(--vp-c-brand-1);
}

.label {
  font-weight: 600;
}

.eyebrow {
  font-size: 0.65rem;
  font-weight: 600;
  letter-spacing: 0.08em;
  text-transform: uppercase;
  opacity: 0.75;
}

.draft {
  margin-top: 0.15rem;
  padding: 0 0.4rem;
  border-radius: 0.5rem;
  background: var(--vp-c-warning-soft);
  color: var(--vp-c-warning-1);
  font-size: 0.7rem;
  font-weight: 600;
}

.separator {
  width: 1px;
  margin: 0.25rem 0.25rem;
  background: var(--vp-c-divider);
}

@media (prefers-reduced-motion: reduce) {
  .tab {
    transition: none;
  }
}
</style>
