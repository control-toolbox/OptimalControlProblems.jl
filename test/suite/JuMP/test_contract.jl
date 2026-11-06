isdefined(Main, :SolveJuMP) ||
    include(joinpath(@__DIR__, "..", "..", "common", "solve_jump.jl"))

module TestContractJuMP

using Test: Test
using JuMP: JuMP
using CTBase: CTBase
using OptimalControlProblems: OptimalControlProblems

const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

# Small grid: only the structure of the model is checked.
const GRID_SIZE = 10

# The schemes of the JuMP models (#369) and their number of controls on `N` steps, written
# here independently of the tableaux: one per node for the trapezoidal rule, one per step
# otherwise.
const SCHEMES = (trapeze=N -> N + 1, euler=N -> N, euler_implicit=N -> N, midpoint=N -> N)
const ORDERINGS = (:interleaved, :blocks)

# Information every JuMP model carries for the solution accessors (`state`, `costate`, ...).
const COMPONENTS = (
    :state_components, :costate_components, :control_components, :variable_components
)

"""
Number of entries of the variable or constraint `name`: the containers of the models are
indexed by the time steps (`0:N`, `0:N-1` or `1:N`), a variable of the problem is a scalar.
"""
function entries(model, name)
    return (c=model[Symbol(name)]; c isa JuMP.AbstractJuMPScalar ? 1 : length(c))
end

"""
Time grid of the model. A free final time is a variable of the model: its value, hence the
grid, exists only after an optimization; zero iterations are run then (no solve).
"""
function time_grid(model)
    try
        return collect(model[:time_grid]())
    catch err
        err isa JuMP.OptimizeNotCalled || rethrow()
        Main.SolveJuMP.solve!(model; max_iter=0)
        return collect(model[:time_grid]())
    end
end

function test_contract()
    N = GRID_SIZE
    Test.@testset "JuMP model contract (no solve)" verbose = VERBOSE showtiming = SHOWTIMING begin
        Main.TestStatus.for_each_problem(
            "contract", OptimalControlProblems.JuMPBackend()
        ) do problem
            build = getfield(OptimalControlProblems, problem)
            for (scheme, controls) in pairs(SCHEMES), ordering in ORDERINGS
                model = build(
                    OptimalControlProblems.JuMPBackend();
                    grid_size=N,
                    scheme=scheme,
                    ordering=ordering,
                )
                names = keys(JuMP.object_dictionary(model))

                Test.@testset "$scheme, $ordering: information" begin
                    Test.@test :time_grid ∈ names
                    Test.@test model[:time_grid] isa Function
                    Test.@test model[:scheme] === scheme
                    Test.@test model[:ordering] === ordering
                    Test.@test length(model[:constraints]) == JuMP.num_constraints(
                        model; count_variable_in_set_constraints=false
                    )
                    for key in COMPONENTS
                        Test.@test key ∈ names
                        Test.@test model[key] isa Vector{String}
                    end
                    Test.@test !isempty(model[:state_components])
                    Test.@test length(model[:costate_components]) ==
                        length(model[:state_components])
                end

                Test.@testset "$scheme, $ordering: names resolve" begin
                    for key in COMPONENTS, name in model[key]
                        Test.@test Symbol(name) ∈ names
                    end
                end

                # state at the N+1 nodes, controls per scheme, one costate (dual of a
                # dynamics constraint) per step
                Test.@testset "$scheme, $ordering: lengths" begin
                    for name in model[:state_components]
                        Test.@test entries(model, name) == N + 1
                    end
                    for name in model[:control_components]
                        Test.@test entries(model, name) == controls(N)
                    end
                    for name in model[:costate_components]
                        Test.@test entries(model, name) == N
                    end
                    for name in model[:variable_components]
                        Test.@test entries(model, name) == 1
                    end
                end

                Test.@testset "$scheme, $ordering: time grid" begin
                    T = time_grid(model)
                    Test.@test length(T) == N + 1
                    Test.@test issorted(T) && T[1] < T[end]
                end
            end

            Test.@testset "unknown scheme or ordering" begin
                Test.@test_throws CTBase.Exceptions.IncorrectArgument build(
                    OptimalControlProblems.JuMPBackend();
                    grid_size=N,
                    scheme=:gauss_legendre_2,
                )
                Test.@test_throws CTBase.Exceptions.IncorrectArgument build(
                    OptimalControlProblems.JuMPBackend(); grid_size=N, ordering=:random
                )
            end
        end
    end
end

end # module TestContractJuMP

test_contract() = TestContractJuMP.test_contract()
