isdefined(Main, :SolveJuMP) || include(joinpath(@__DIR__, "..", "..", "common", "solve_jump.jl"))

module TestContractJuMP

using Test: Test
using JuMP: JuMP
using OptimalControlProblems: OptimalControlProblems

const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

# Small grid: only the structure of the model is checked.
const GRID_SIZE = 10

# Information every JuMP model carries for the solution accessors (`state`, `costate`, ...).
# `:scheme` joins this list with #284 (schemes of the JuMP models).
const COMPONENTS = (
    :state_components, :costate_components, :control_components, :variable_components
)

"""
Number of entries of the variable or constraint `name`: the containers of the models are
indexed by the time steps (`0:N` or `1:N`), a variable of the problem is a scalar.
"""
entries(model, name) = (c = model[Symbol(name)]; c isa JuMP.AbstractJuMPScalar ? 1 : length(c))

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
        Main.TestStatus.for_each_problem("contract", OptimalControlProblems.JuMPBackend()) do problem
            build = getfield(OptimalControlProblems, problem)
            model = build(OptimalControlProblems.JuMPBackend(); grid_size=N)
            names = keys(JuMP.object_dictionary(model))

            Test.@testset "information" begin
                Test.@test :time_grid ∈ names
                Test.@test model[:time_grid] isa Function
                for key in COMPONENTS
                    Test.@test key ∈ names
                    Test.@test model[key] isa Vector{String}
                end
                Test.@test !isempty(model[:state_components])
                Test.@test length(model[:costate_components]) == length(model[:state_components])
            end

            Test.@testset "names resolve" begin
                for key in COMPONENTS, name in model[key]
                    Test.@test Symbol(name) ∈ names
                end
            end

            # trapezoidal scheme: state and control at the N+1 points of the grid, one
            # costate (dual of a dynamics constraint) per step
            Test.@testset "lengths" begin
                for name in model[:state_components]
                    Test.@test entries(model, name) == N + 1
                end
                for name in model[:control_components]
                    Test.@test entries(model, name) == N + 1
                end
                for name in model[:costate_components]
                    Test.@test entries(model, name) == N
                end
                for name in model[:variable_components]
                    Test.@test entries(model, name) == 1
                end
            end

            Test.@testset "time grid" begin
                T = time_grid(model)
                Test.@test length(T) == N + 1
                Test.@test issorted(T) && T[1] < T[end]
            end
        end
    end
end

end # module TestContractJuMP

test_contract() = TestContractJuMP.test_contract()
