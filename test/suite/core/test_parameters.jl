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
        Test.@test_throws CTBase.UnauthorizedCall OptimalControlProblems.merge(
            nothing, (tf=1,)
        )
    end
end

end # module TestParameters

test_parameters() = TestParameters.test_parameters()
