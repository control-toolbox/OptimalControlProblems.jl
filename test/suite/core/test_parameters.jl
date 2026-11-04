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
        data = OptimalControlProblems.problem_metadata(:beam)
        Test.@test data[:grid_size] == OptimalControlProblems.grid_size_data(:beam)
        Test.@test data[:parameters] == OptimalControlProblems.parameters_data(:beam)
        Test.@test OptimalControlProblems.problem_metadata()[:beam] === data
        Test.@test_throws CTBase.Exceptions.IncorrectArgument OptimalControlProblems.problem_metadata(
            :not_a_problem
        )
    end
end

end # module TestParameters

test_parameters() = TestParameters.test_parameters()
