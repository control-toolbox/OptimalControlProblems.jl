"""
The metadata of the problems: the list of the problems (one file per problem in
`ext/MetaData/`), the schema of a metadata file, loading and checking every file, and reading
them: `problem_metadata`, `grid_size_data`, `parameters_data`, `initial_guess_data`. The
analytic solutions used by some metadata files (`reference_solution`) are defined here too.

Depends on no other submodule.
"""
module Metadata

using CTBase: CTBase
using DocStringExtensions: TYPEDSIGNATURES
using OrderedCollections: OrderedDict

include(joinpath(@__DIR__, "discovery.jl"))
include(joinpath(@__DIR__, "solutions.jl"))
include(joinpath(@__DIR__, "loading.jl"))
include(joinpath(@__DIR__, "access.jl"))

export problem_metadata, grid_size_data, parameters_data, initial_guess_data

end # module Metadata
