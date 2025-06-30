using Aqua
using CTBase
using Ipopt
using JuMP
using NLPModelsIpopt
using OptimalControl
using OptimalControlProblems
using Test
using Plots

#
using Interpolations
include("utils.jl") 

# Parameters
const tol = 1e-6
const mu_strategy = "adaptive"
const sb = "yes"
const constr_viol_tol = 1e-6
const max_iter = 1000
const max_wall_time = 500.0

# Collecting all the OptimalControlProblems
all_names = names(OptimalControlProblems; all=true)
functions_list = filter(
    x ->
        isdefined(OptimalControlProblems, x) &&
            isa(getfield(OptimalControlProblems, x), Function) &&
            !startswith(string(x), "#") &&
            !(x in [:eval, :include]),
    all_names,
)

pbs_with_issues = [
    :glider, :moonlander,               # issues with OptimalControl
    :cart_pendulum, :truck_trailer,     # issues with JuMP
    :space_shuttle,                     # not the same probleme between JuMP (tf not fixed) and OptimalControl (tf fixed)
]
functions_list = setdiff(functions_list, pbs_with_issues)

const list_of_problems = deepcopy(functions_list)
list_of_problems_final = deepcopy(functions_list)

#
const verbose = true
@testset "OptimalControlProblems tests" verbose=verbose showtiming=true begin
    for name in (
        # :aqua, 
        # :JuMP, 
        # :OptimalControl,
        # :Comparison,
        :Init,
        )
        @testset "$(name)" verbose=verbose begin
            test_name = Symbol(:test_, name)
            println("Testing: " * string(name))
            include("$(test_name).jl")
            @eval $test_name()
        end
    end
end

#
println("List of problems working:", list_of_problems_final)

# Sauvegarder la liste des problèmes fonctionnels dans un fichier de cache
cache_file = joinpath(@__DIR__, "..", "available_problems_cache.txt")
try
    open(cache_file, "w") do f
        for problem in list_of_problems_final
            println(f, string(problem))
        end
    end
    println("Cache des problèmes disponibles mis à jour: $cache_file")
catch e
    @warn "Impossible de sauvegarder le cache des problèmes: $e"
end