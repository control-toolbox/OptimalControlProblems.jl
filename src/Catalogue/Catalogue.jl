"""
The catalogue: one function per problem (`beam`, `chain`, …), whose methods for each
modeling are defined by the extensions, the list of the problems available in some
modelings with some status (`problems`), and the status of a problem in each modeling
(`status`).

Depends on `Modelings` and `Metadata`.
"""
module Catalogue

using CTBase: CTBase
using DocStringExtensions: TYPEDSIGNATURES

using ..Modelings: Modelings
using ..Metadata: Metadata

include(joinpath(@__DIR__, "selection.jl"))
include(joinpath(@__DIR__, "functions.jl"))

export problems, status
# the problem functions, one per metadata file: the list is the files, not written here
eval(Expr(:export, Metadata.PROBLEM_NAMES...))

end # module Catalogue
