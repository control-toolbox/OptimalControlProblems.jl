<script setup lang="ts">
import { computed } from 'vue'
import { currentView } from './problemView'

// One view of a problem page. The panel marked `default` is the one the server renders and
// the one shown before the tab bar has chosen.
const props = defineProps<{ id: string; default?: boolean }>()

const visible = computed(() =>
  currentView.value === '' ? props.default === true : currentView.value === props.id,
)
</script>

<template>
  <section
    :id="`view-${id}`"
    class="problem-panel"
    role="tabpanel"
    :aria-labelledby="`tab-${id}`"
    v-show="visible"
  >
    <slot />
  </section>
</template>
