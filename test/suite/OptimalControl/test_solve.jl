isdefined(Main, :SolveOptimalControl) ||
    include(joinpath(@__DIR__, "..", "..", "common", "solve_optimalcontrol.jl"))

module TestSolveOptimalControl

using Test: Test
using OptimalControl: OptimalControl # loads the OptimalControlModels extension
using OptimalControlProblems: OptimalControlProblems

const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

function test_solve()
    S = Main.SolveOptimalControl
    backend = OptimalControlProblems.OptimalControlBackend()
    Test.@testset "OptimalControl solve (small grid)" verbose = VERBOSE showtiming =
        SHOWTIMING begin
        for config in Main.TestConfigs.CONFIGS.OptimalControl
            Test.@testset "$(Main.TestConfigs.label(config))" begin
                test = "test_solve $(Main.TestConfigs.label(config))"
                Main.TestStatus.for_each_problem(test, backend) do problem
                    pb = getfield(OptimalControlProblems, problem)(backend)
                    N = Main.TestConfigs.SMALL_GRID
                    sol = S.solve(pb, config; grid_size=N)
                    Test.@test S.succeeded(sol)
                end
            end
        end
    end
end

end # module TestSolveOptimalControl

test_solve() = TestSolveOptimalControl.test_solve()
