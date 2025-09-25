module OptimalControlModels

using OptimalControlProblems
using OptimalControl
using DocStringExtensions
using OrderedCollections: OrderedDict

# list of problems
list_of_problems = OptimalControlProblems.problems()

# include problems files
rel_path_problems = "OptimalControlModels"
path = joinpath(dirname(@__FILE__), rel_path_problems)
for problem in list_of_problems
    include(joinpath(rel_path_problems, "$(problem).jl"))
end

# include problems files (_s versions)
rel_path_problems = "OptimalControlModels_s"
path = joinpath(dirname(@__FILE__), rel_path_problems)
for problem in list_of_problems
    include(joinpath(rel_path_problems, "$(problem)_s.jl"))
end

end
