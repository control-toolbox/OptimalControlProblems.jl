module OptimalControlModels

using OptimalControlProblems
using OptimalControl
using DocStringExtensions
using OrderedCollections: OrderedDict

# Include only files belonging to this modeling. The file presence is the
# source of truth for availability.
rel_path_problems = "OptimalControlModels"
for problem in OptimalControlProblems.problems(OptimalControlBackend(); status=:all)
    include(joinpath(rel_path_problems, "$(problem).jl"))
end

# Scalarized OptimalControl problems use a distinct backend and directory.
rel_path_problems = "OptimalControlScalarizedModels"
for problem in OptimalControlProblems.problems(OptimalControlScalarizedBackend(); status=:all)
    include(joinpath(rel_path_problems, "$(problem).jl"))
end

end
