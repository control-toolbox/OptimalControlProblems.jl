# Accessors shared with OptimalControl, which reexports these very functions: binding them
# here, instead of defining our own, lets `using OptimalControl, OptimalControlProblems`
# run without a name conflict, and `state(jump_model)` and `state(sol)` call one function.
const time_grid = CTModels.Components.time_grid
const state = CTModels.Components.state
const control = CTModels.Components.control
const costate = CTModels.Components.costate
const variable = CTModels.Components.variable
const objective = CTModels.Components.objective
const iterations = CTModels.Solutions.iterations
const state_components = CTModels.Models.state_components
const control_components = CTModels.Models.control_components
const variable_components = CTModels.Models.variable_components
const state_dimension = CTModels.Models.state_dimension
const control_dimension = CTModels.Models.control_dimension
const variable_dimension = CTModels.Models.variable_dimension
const metadata = CTBase.Strategies.metadata
const model = CTModels.Solutions.model
