module OCPDocumentation

using HTTP
using JSON
using OptimalControlProblems
using DocStringExtensions

include(joinpath("OCPDocumentation", "prompt.jl"))
include(joinpath("OCPDocumentation", "app.jl"))

end