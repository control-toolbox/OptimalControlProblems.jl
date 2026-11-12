module TestParameters

using Test: Test
using CTBase: CTBase
using OptimalControlProblems: OptimalControlProblems

const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

function test_parameters()
    Test.@testset "merge (parameters)" verbose = VERBOSE showtiming = SHOWTIMING begin
        Test.@test OptimalControlProblems.Metadata.merge(nothing, nothing) === nothing
        Test.@test OptimalControlProblems.Metadata.merge((tf=1,), nothing) == (tf=1,)
        Test.@test OptimalControlProblems.Metadata.merge((tf=1,), (tf=2,)) == (tf=2,)
        Test.@test OptimalControlProblems.Metadata.merge((tf=1, a=2), (tf=2,)) ==
            (tf=2, a=2)
        Test.@test_throws CTBase.Exceptions.PreconditionError OptimalControlProblems.Metadata.merge(
            nothing, (tf=1,)
        )
    end
    Test.@testset "problem_metadata" verbose = VERBOSE showtiming = SHOWTIMING begin
        problem = first(OptimalControlProblems.problems(; status=:all))
        data = OptimalControlProblems.problem_metadata(problem)
        Test.@test data[:grid_size] == OptimalControlProblems.grid_size_data(problem)
        Test.@test data[:parameters] == OptimalControlProblems.parameters_data(problem)
        # the initial guess comes from the parameters (#377)
        guess = OptimalControlProblems.initial_guess_data(problem)
        Test.@test guess isa NamedTuple
        Test.@test issubset(keys(guess), (:state, :control, :variable))
        Test.@test guess == data[:init](OptimalControlProblems.parameters_data(problem))
        Test.@test OptimalControlProblems.problem_metadata()[problem] === data
        Test.@test_throws CTBase.Exceptions.IncorrectArgument OptimalControlProblems.problem_metadata(
            :not_a_problem
        )
    end
    Test.@testset "reference_solution_data" verbose = VERBOSE showtiming = SHOWTIMING begin
        for problem in OptimalControlProblems.problems(; status=:all)
            data = OptimalControlProblems.problem_metadata(problem)
            solution = OptimalControlProblems.reference_solution_data(problem)
            if haskey(data, :reference_solution)
                # the default parameters, given or not
                expected = data[:reference_solution](data[:parameters])
                Test.@test solution.objective == expected.objective
                given = OptimalControlProblems.reference_solution_data(
                    problem, data[:parameters]
                )
                Test.@test given.objective == expected.objective
            else
                Test.@test isnothing(solution)
            end
        end
    end
    Test.@testset "Bryson–Denham solution" verbose = VERBOSE showtiming = SHOWTIMING begin
        solve(ℓ; t0=0, tf=1, x_t0=(0, 1), x_tf=(0, -1), weight=1, kwargs...) =
            OptimalControlProblems.Metadata._bryson_denham_solution(
                t0, tf, x_t0, x_tf, ℓ; weight, kwargs...
            )
        solution = solve(0.1)
        Test.@test solution.objective ≈ 8 / 0.9
        Test.@test solve(0.1; weight=0.5).objective ≈ 4 / 0.9
        Test.@test solution.state(0) == [0, 1] && solution.state(1) == [0, -1]
        Test.@test solution.state(0.4) ≈ [0.1, 0] && solution.control(0.4) == 0
        Test.@test solution.state(0.5) == [0.1, 0] && solution.control(0.6) == 0
        Test.@test solution.control(0) ≈ -2 / 0.3 && solution.control(1) ≈ -2 / 0.3
        # known only with a boundary arc, for these boundary conditions
        Test.@test isnothing(solve(0.2))
        Test.@test isnothing(solve(0.1; tf=2))
        Test.@test isnothing(solve(0.1; x_t0=(0, 2)))
        Test.@test isnothing(solve(0.1; lower=0.05))
        Test.@test !isnothing(solve(0.1; lower=0))
    end
end

end # module TestParameters

test_parameters() = TestParameters.test_parameters()
