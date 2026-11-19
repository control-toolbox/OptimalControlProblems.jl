isdefined(Main, :SolveJuMP) ||
    include(joinpath(@__DIR__, "..", "..", "common", "solve_jump.jl"))

module TestSmokeJuMP

using Test: Test
using JuMP: JuMP
using OptimalControlProblems: OptimalControlProblems

const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

# Every problem builds, evaluates and runs a few iterations; the convergence is reported by
# the convergence tests (`convergence` argument, cron), #360.
function test_smoke()
    Test.@testset "JuMP smoke (small grid, few iterations)" verbose = VERBOSE showtiming =
        SHOWTIMING begin
        for config in Main.TestConfigs.CONFIGS.JuMP
            Test.@testset "$(Main.TestConfigs.label(config))" begin
                test = "test_smoke $(Main.TestConfigs.label(config))"
                Main.TestStatus.for_each_problem(
                    test, OptimalControlProblems.JuMPBackend()
                ) do problem
                    build = getfield(OptimalControlProblems, problem)
                    o = Main.TestConfigs
                    model = build(
                        OptimalControlProblems.JuMPBackend(); grid_size=o.SMOKE_GRID
                    )
                    Main.SolveJuMP.solve!(
                        model; solver=config.solver, max_iter=o.SMOKE_MAX_ITER
                    )
                    Test.@test JuMP.has_values(model) &&
                        isfinite(JuMP.objective_value(model))
                end
            end
        end
    end
end

end # module TestSmokeJuMP

test_smoke() = TestSmokeJuMP.test_smoke()
