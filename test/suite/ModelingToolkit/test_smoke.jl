isdefined(Main, :SolveModelingToolkit) ||
    include(joinpath(@__DIR__, "..", "..", "common", "solve_modelingtoolkit.jl"))

module TestSmokeModelingToolkit

using Test: Test
using ModelingToolkit: ModelingToolkit as MTK
using OptimalControlProblems: OptimalControlProblems

const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

# Every problem builds, evaluates and runs a few iterations in each configuration; the
# convergence is reported by the convergence tests.
function test_smoke()
    S = Main.SolveModelingToolkit
    backend = OptimalControlProblems.ModelingToolkitBackend()
    Test.@testset "ModelingToolkit smoke (small grid, few iterations)" verbose = VERBOSE showtiming =
        SHOWTIMING begin
        for config in Main.TestConfigs.CONFIGS.ModelingToolkit
            Test.@testset "$(Main.TestConfigs.label(config))" begin
                test = "test_smoke $(Main.TestConfigs.label(config))"
                Main.TestStatus.for_each_problem(test, backend) do problem
                    pb = getfield(OptimalControlProblems, problem)(backend)
                    o = Main.TestConfigs
                    sol = S.solve(pb, config; grid_size=o.SMOKE_GRID, max_iter=o.SMOKE_MAX_ITER)
                    Test.@test isfinite(sol.objective)
                end
            end
        end
    end
end

end # module TestSmokeModelingToolkit

test_smoke() = TestSmokeModelingToolkit.test_smoke()
