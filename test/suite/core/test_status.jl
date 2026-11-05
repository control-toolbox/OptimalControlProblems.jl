module TestStatusLoop

using Test: Test
using OptimalControlProblems: OptimalControlProblems

const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

const BACKEND = OptimalControlProblems.JuMPBackend()

"""Run `f` with the problem selection `names`, then restore the previous one."""
function with_selection(f, names)
    previous = Main.TestConfigs.PROBLEMS[]
    try
        Main.TestStatus.select_problems!(names)
        return f()
    finally
        Main.TestConfigs.PROBLEMS[] = previous
    end
end

"""Problems visited by `for_each_problem` (the test bodies do nothing)."""
function visited()
    seen = Symbol[]
    Main.TestStatus.for_each_problem(problem -> push!(seen, problem), "visited", BACKEND)
    return seen
end

function test_status()
    Test.@testset "status loop and problem selection" verbose = VERBOSE showtiming = SHOWTIMING begin
        all_problems = OptimalControlProblems.problems(BACKEND; status=:all)
        # whatever the selection of this run (`problem=…`), the loop without selection visits all
        Test.@test with_selection(visited, String[]) == all_problems

        Test.@testset "selection" begin
            one = first(all_problems)
            Test.@test with_selection(visited, [string(one)]) == [one]
            # a selection restricts, it does not extend: a problem absent from the modeling
            # is not visited
            absent = setdiff(OptimalControlProblems.problems(; status=:all), all_problems)
            isempty(absent) || Test.@test with_selection(visited, [string(first(absent))]) == Symbol[]
            # an unknown name is an error: a selection matching nothing would test nothing
            Test.@test_throws ArgumentError Main.TestStatus.select_problems!(["not_a_problem"])
            # no name: no restriction
            Test.@test with_selection(visited, String[]) == all_problems
        end

        Test.@testset "drafts do not fail the run" begin
            # a draft problem whose test fails is recorded, not reported to the parent
            # (`@testset` takes a name, not a qualified one, on Julia 1.10)
            DraftTestSet = Main.TestStatus.DraftTestSet
            Test.@test DraftTestSet("draft").fail == 0
            ts = Test.@testset DraftTestSet "inner" begin
                Test.@test false
                Test.@test true
            end
            Test.@test (ts.pass, ts.fail) == (1, 1)
            Test.@test Main.TestStatus.outcome(ts) == "fail"
        end
    end
end

end # module TestStatusLoop

test_status() = TestStatusLoop.test_status()
