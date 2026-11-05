isdefined(Main, :Comparison) || include(joinpath(@__DIR__, "..", "..", "common", "comparison.jl"))

module TestSolutionComparison

using Test: Test
using CTBase: CTBase
using JuMP: JuMP
using OptimalControl: OptimalControl
using OptimalControlProblems: OptimalControlProblems

const OCP = OptimalControlProblems
const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

vec1(a::Number) = [Float64(a)]
vec1(a) = Float64.(a)

# The solution built from a JuMP model gives back what the JuMP accessors give. Zero
# iterations: the structure, not a solve (the solves are compared below, on `beam`).
function check_solution(problem::Symbol)
    model = getfield(OCP, problem)(OCP.JuMPBackend(); grid_size=Main.TestConfigs.SMALL_GRID)
    Main.SolveJuMP.solve!(model; max_iter=0)
    pb = getfield(OCP, problem)(OCP.OptimalControlBackend())
    sol = OCP.solution(model, pb)

    T = collect(Float64, OCP.time_grid(model))
    Test.@test OCP.model(sol) === OCP.model(pb)
    Test.@test collect(Float64, OCP.time_grid(sol)) == T
    Test.@test OCP.objective(sol) == OCP.objective(model)
    Test.@test OCP.iterations(sol) == OCP.iterations(model)
    Test.@test vec1(OCP.variable(sol)) == vec1(OCP.variable(model))
    Test.@test all(t -> vec1(OCP.state(sol)(t)) ≈ vec1(OCP.state(model)(t)), T)
    Test.@test all(t -> vec1(OCP.control(sol)(t)) ≈ vec1(OCP.control(model)(t)), T)
    Test.@test all(t -> vec1(OCP.costate(sol)(t)) ≈ vec1(OCP.costate(model)(t)), T[1:(end - 1)])
    return nothing
end

function test_solution()
    Test.@testset "JuMP solution as a CTModels.Solution" verbose = VERBOSE showtiming = SHOWTIMING begin
        Main.TestStatus.for_each_problem(
            "test_solution", OCP.JuMPBackend(), OCP.OptimalControlBackend()
        ) do problem
            check_solution(problem)
        end

        Test.@testset "errors" begin
            pb = OCP.beam(OCP.OptimalControlBackend())
            # not solved
            unsolved = OCP.beam(OCP.JuMPBackend(); grid_size=Main.TestConfigs.SMALL_GRID)
            Test.@test_throws CTBase.Exceptions.PreconditionError OCP.solution(unsolved, pb)
            # a problem of another dimension
            model = OCP.beam(OCP.JuMPBackend(); grid_size=Main.TestConfigs.SMALL_GRID)
            Main.SolveJuMP.solve!(model; max_iter=0)
            other = OCP.glider(OCP.OptimalControlBackend())
            Test.@test_throws CTBase.Exceptions.IncorrectArgument OCP.solution(model, other)
        end

        Test.@testset "beam, solved, against OptimalControl" begin
            N = Main.TestConfigs.SMALL_GRID
            model = OCP.beam(OCP.JuMPBackend(); grid_size=N)
            Main.SolveJuMP.solve!(model)
            pb = OCP.beam(OCP.OptimalControlBackend())
            sol = OCP.solution(model, pb)
            Test.@test OptimalControl.successful(sol)

            ref = Main.SolveOptimalControl.solve(
                pb, first(Main.TestConfigs.CONFIGS.OptimalControl); grid_size=N
            )
            C = Main.Comparison
            Test.@test C.within(OCP.objective(sol), OCP.objective(ref); C.OBJECTIVE...)
            T = collect(Float64, OCP.time_grid(sol))
            for i in eachindex(first(vec1(OCP.state(ref)(0.0))))
                a = [vec1(OCP.state(sol)(t))[i] for t in T]
                b = [vec1(OCP.state(ref)(t))[i] for t in T]
                Test.@test C.within_L2(T, a, b; C.STATE...)
            end
        end
    end
end

end # module TestSolutionComparison

test_solution() = TestSolutionComparison.test_solution()
