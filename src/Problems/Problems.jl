"""
A problem as the OptimalControl modelings return it (`Problem`, read with `model`,
`initial_guess` and `metadata`), its computed `features`, the `solution` of a solved model of
another modeling, and the accessors shared with OptimalControl (`state`, `control`,
`objective`, …).

Depends on `Modelings`, `Metadata` and `Catalogue`.
"""
module Problems

using CTBase: CTBase
using CTModels: CTModels
using DocStringExtensions: TYPEDEF, TYPEDFIELDS, TYPEDSIGNATURES
using OrderedCollections: OrderedDict

using ..Modelings: Modelings
using ..Metadata: Metadata
using ..Catalogue: Catalogue

include(joinpath(@__DIR__, "aliases.jl"))
include(joinpath(@__DIR__, "problem.jl"))

export Problem, model, initial_guess, metadata, solution, features
export time_grid, state, costate, control, variable, iterations, objective
export control_components,
    control_dimension,
    state_components,
    state_dimension,
    variable_components,
    variable_dimension

end # module Problems
