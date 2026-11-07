// The view shown on a problem page: the problem itself, one of its modelings, or the
// comparison of the modelings. Shared by the tab bar (`ProblemTabs`) and the panels
// (`ProblemPanel`) of the page. Empty until the tab bar has mounted: the panels then show
// their default, which is also what the server renders.
import { ref } from 'vue'

export const currentView = ref('')

// The last view chosen, remembered in the browser so that a reader of the JuMP models sees
// JuMP first on every problem page. A per-viewer convenience: the page works without it.
const STORAGE_KEY = 'ocp.problem.view'

export function rememberedView(): string | null {
  try {
    return window.localStorage.getItem(STORAGE_KEY)
  } catch {
    return null
  }
}

export function rememberView(view: string): void {
  try {
    window.localStorage.setItem(STORAGE_KEY, view)
  } catch {
    // private window or blocked storage: nothing to remember
  }
}
