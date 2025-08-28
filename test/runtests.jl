using Aqua
using CTBase
using CTDirect
using Ipopt
using JuMP
using NLPModelsIpopt
using OptimalControl
using OptimalControlProblems
using Test
using Plots
using Plots.PlotMeasures # for leftmargin, bottommargin
using Interpolations
include("utils.jl")

# Parameters for the solvers
const TOL = 1e-8
const MU_STRATEGY = "adaptive"
const SB = "yes"
const MAX_ITER = 1000
const MAX_WALL_TIME = 500.0

# Collect all the problems from OptimalControlProblems
path = joinpath(dirname(@__FILE__), "..", "ext", "MetaData")
list_of_problems = []
files = filter(x -> x[(end - 2):end] == ".jl", readdir(path))
for file in files
    problem = Symbol(file[1:(end - 3)])
    push!(list_of_problems, problem)
end

# Remove from the tests the following problems
# problems_to_exclude = [

# ]
# list_of_problems = setdiff(list_of_problems, problems_to_exclude)

list_of_problems = [
#    :beam
]

# The list of all the problems to test
const LIST_OF_PROBLEMS = deepcopy(list_of_problems)

# The final list of problems for which the tests pass
LIST_OF_PROBLEMS_FINAL = deepcopy(list_of_problems)

# Tests
const DEBUG = true
const VERBOSE = true # print or not details during tests
@testset "OptimalControlProblems tests" verbose=VERBOSE showtiming=true begin
    for name in (
        #:aqua,
        :kwargs,
        :JuMP,                  # convergence tests for JuMP models
        :OptimalControl,        # convergence tests for OptimalControl models
        :init,                  # comparison between OptimalControl and JuMP: init
        :solution,              # comparison between OptimalControl and JuMP: solution
        :quick,                 # quick comparison: objective rel error only
    )
        @testset "$(name)" verbose=VERBOSE begin
            test_name = Symbol(:test_, name)
            println("Testing: " * string(name))
            include("$(test_name).jl")
            @eval $test_name()
        end
    end

    # compare the list of problems that passed the tests to the available problems
    println("\nProblems that passed the tests: ");
    display(LIST_OF_PROBLEMS_FINAL)
    println("\nList of available problems: ");
    display(problems());
    println()

    @testset "available_problems" verbose=VERBOSE begin
        @test LIST_OF_PROBLEMS_FINAL == problems()
    end
end
