"""
A collection of optimal control problems, each available in one or more modelings
(OptimalControl, its scalarized form, JuMP), loaded by the extensions.

The code is split into submodules (ct-conventions `MOD1`, `MOD2`), in dependency order:
`Modelings`, `Metadata`, `References`, `Catalogue`, `Problems`. The package re-exports their
public API, which is the API of the package; what a submodule does not export is reached
through it, e.g. `OptimalControlProblems.Metadata.METADATA_SCHEMA`. See `src/README.md`.
"""
module OptimalControlProblems

include(joinpath(@__DIR__, "Modelings", "Modelings.jl"))
using .Modelings
include(joinpath(@__DIR__, "Metadata", "Metadata.jl"))
using .Metadata
include(joinpath(@__DIR__, "References", "References.jl"))
using .References
include(joinpath(@__DIR__, "Catalogue", "Catalogue.jl"))
using .Catalogue
include(joinpath(@__DIR__, "Problems", "Problems.jl"))
using .Problems

# The package is used directly, unlike the internal CT packages: it exports its main API
# (#413). The other public functions (`status`, `features`, `reference_objective`, …) are
# called qualified, `OptimalControlProblems.status(problem)`.
export JuMPBackend,
    ModelingToolkitBackend, OptimalControlBackend, OptimalControlScalarizedBackend, problems
export time_grid, state, costate, control, variable, iterations, objective
export control_components,
    control_dimension,
    state_components,
    state_dimension,
    variable_components,
    variable_dimension
export problem_metadata, grid_size_data, parameters_data
export Problem, model, initial_guess, metadata, solution
# the problem functions (`Catalogue`), one per metadata file
eval(Expr(:export, Metadata.PROBLEM_NAMES...))

end # module OptimalControlProblems
