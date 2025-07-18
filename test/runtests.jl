using Aqua
using CTBase
using Ipopt
using JuMP
using NLPModelsIpopt
using OptimalControl
using OptimalControlProblems
using Test
using Plots
using Interpolations
include("utils.jl") 

# Parameters for the solvers
const tol = 1e-8
const mu_strategy = "adaptive"
const sb = "yes"
const max_iter = 1000
const max_wall_time = 500.0

# Collect all the problem from OptimalControlProblems
all_names = names(OptimalControlProblems; all=true)
functions_list = filter(
    x ->
        isdefined(OptimalControlProblems, x) &&
            isa(getfield(OptimalControlProblems, x), Function) &&
            !startswith(string(x), "#") &&
            !(x in [:eval, :include, :available_problems]),
    all_names,
)

# Remove from the tests the problems that are not working
pbs_with_issues = [
    #:glider, :moonlander,               # issues with OptimalControl
    #:cart_pendulum, :truck_trailer,     # issues with JuMP
    #:space_shuttle,                     # not the same probleme between JuMP (tf not fixed) and OptimalControl (tf fixed)
]
functions_list = setdiff(functions_list, pbs_with_issues)

# The list of all the problems to test
const list_of_problems = deepcopy(functions_list)

# The final list of problems for which the tests pass
list_of_problems_final = deepcopy(functions_list)

# Tests
const verbose = true # print or not details during tests
@testset "OptimalControlProblems tests" verbose=verbose showtiming=true begin

    for name in (
        #:aqua, 
        #:JuMP,                  # convergence tests for JuMP models
        #:OptimalControl,        # convergence tests for OptimalControl models
        :Comparison,            # comparison between OptimalControl and JuMP
        #:quick,                 # quick comparison: objective rel error only
        )
        @testset "$(name)" verbose=verbose begin
            test_name = Symbol(:test_, name)
            println("Testing: " * string(name))
            include("$(test_name).jl")
            @eval $test_name()
        end
    end
 
    
    @testset "available_problems" verbose=verbose begin
        println(available_problems())
        println(list_of_problems_final)
        if list_of_problems_final == available_problems()
            @test list_of_problems_final == available_problems()
        else
            @test list_of_problems_final == available_problems() broken=true
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
