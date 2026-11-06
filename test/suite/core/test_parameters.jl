module TestParameters

using Test: Test
using CTBase: CTBase
using OptimalControlProblems: OptimalControlProblems

const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

function test_parameters()
    Test.@testset "merge (parameters)" verbose = VERBOSE showtiming = SHOWTIMING begin
        Test.@test OptimalControlProblems.merge(nothing, nothing) === nothing
        Test.@test OptimalControlProblems.merge((tf=1,), nothing) == (tf=1,)
        Test.@test OptimalControlProblems.merge((tf=1,), (tf=2,)) == (tf=2,)
        Test.@test OptimalControlProblems.merge((tf=1, a=2), (tf=2,)) == (tf=2, a=2)
        Test.@test_throws CTBase.Exceptions.PreconditionError OptimalControlProblems.merge(
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
end

end # module TestParameters

test_parameters() = TestParameters.test_parameters()
