isdefined(Main, :SolveOptimalControl) ||
    include(joinpath(@__DIR__, "..", "..", "common", "solve_optimalcontrol.jl"))

module TestSmokeOptimalControlScalarized

using Test: Test
using OptimalControl: OptimalControl # loads the OptimalControlModels extension
using OptimalControlProblems: OptimalControlProblems

const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

# Every problem builds, evaluates and runs a few iterations in each configuration; the
# convergence is reported by the convergence tests (`convergence` argument, cron), #360.
function test_smoke()
    S = Main.SolveOptimalControl
    backend = OptimalControlProblems.OptimalControlScalarizedBackend()
    Test.@testset "OptimalControlScalarized smoke (small grid, few iterations)" verbose =
        VERBOSE showtiming = SHOWTIMING begin
        for config in Main.TestConfigs.CONFIGS.OptimalControlScalarized
            Test.@testset "$(Main.TestConfigs.label(config))" begin
                test = "test_smoke $(Main.TestConfigs.label(config))"
                Main.TestStatus.for_each_problem(test, backend) do problem
                    pb = getfield(OptimalControlProblems, problem)(backend)
                    o = Main.TestConfigs
                    sol = S.solve(
                        pb, config; grid_size=o.SMOKE_GRID, max_iter=o.SMOKE_MAX_ITER
                    )
                    Test.@test isfinite(OptimalControl.objective(sol))
                end
            end
        end
    end
end

end # module TestSmokeOptimalControlScalarized

test_smoke() = TestSmokeOptimalControlScalarized.test_smoke()
