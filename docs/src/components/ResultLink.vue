<script setup lang="ts">
// A file of the results of the crons, served with the site (`docs/results.jl` copies them to
// `public/results/`): a link that follows the base of the deployed version.
const props = defineProps<{ path: string }>()
const href = `${import.meta.env.BASE_URL}results/${props.path}`
</script>

<template>
  <a :href="href" target="_blank" rel="noopener"><slot /></a>
</template>
