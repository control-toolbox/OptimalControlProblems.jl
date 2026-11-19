module TestDraftJuMP

using Test: Test
using JuMP: JuMP
using OptimalControlProblems: OptimalControlProblems

const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

function test_draft()
    Test.@testset "JuMP draft warning" verbose = VERBOSE showtiming = SHOWTIMING begin
        problem = first(
            OptimalControlProblems.problems(OptimalControlProblems.JuMPBackend())
        )
        build = getfield(OptimalControlProblems, problem)
        data = OptimalControlProblems.problem_metadata(problem)
        original = data[:status]
        # no warning for a valid problem
        Test.@test_logs build(OptimalControlProblems.JuMPBackend(); grid_size=2)
        data[:status] = merge(original, (JuMP=:draft,))
        try
            Test.@test_logs (:warn, Regex("$problem is a draft for JuMP")) match_mode = :any build(
                OptimalControlProblems.JuMPBackend(); grid_size=2
            )
        finally
            data[:status] = original
        end
    end
end

end # module TestDraftJuMP

test_draft() = TestDraftJuMP.test_draft()
