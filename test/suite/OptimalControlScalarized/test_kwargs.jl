module TestKwargsOptimalControlScalarized

using Test: Test
using CTDirect: CTDirect
using OptimalControl: OptimalControl # loads the OptimalControlModels extension
using OptimalControlProblems: OptimalControlProblems

const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

function test_kwargs()
    grid_size = 2
    Test.@testset "OptimalControlScalarized keyword arguments" verbose = VERBOSE showtiming = SHOWTIMING begin
        for problem in OptimalControlProblems.problems(OptimalControlProblems.OptimalControlScalarizedBackend())
            Test.@testset "$problem" begin
                # transitional: the scalarized variants are the `_s` functions until #303
                build = getfield(OptimalControlProblems, problem)
                docp = build(
                    OptimalControlProblems.OptimalControlScalarizedBackend(),
                    :madnlp,
                    :exa;
                    grid_size=grid_size,
                    disc_method=:euler,
                )
                Test.@test docp isa CTDirect.DOCP
                Test.@test docp.time.steps == grid_size
                Test.@test docp.discretization isa CTDirect.Euler
            end
        end
    end
end

end # module TestKwargsOptimalControlScalarized

test_kwargs() = TestKwargsOptimalControlScalarized.test_kwargs()
