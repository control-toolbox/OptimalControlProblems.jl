"""
The modelings of the collection: the backend types that select one (`JuMPBackend`,
`OptimalControlBackend`, `OptimalControlScalarizedBackend`) and the modeling table, the
single place where a modeling is declared (its name, its backend, the directory of its
models in `ext/`, the weak dependency that loads them).

Depends on no other submodule.
"""
module Modelings

using CTBase: CTBase
using DocStringExtensions: TYPEDEF

include(joinpath(@__DIR__, "backends.jl"))

export AbstractModelBackend,
    JuMPBackend,
    ModelingToolkitBackend,
    OptimalControlBackend,
    OptimalControlScalarizedBackend

end # module Modelings
