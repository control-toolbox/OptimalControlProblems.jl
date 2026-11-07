isdefined(Main, :SolveOptimalControl) ||
    include(joinpath(@__DIR__, "..", "..", "common", "solve_optimalcontrol.jl"))

module TestRobustnessOptimalControl

using Test: Test
using OptimalControl: OptimalControl # loads the OptimalControlModels extension
using OptimalControlProblems: OptimalControlProblems

const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

# Robustness of the initial guess (#378): every problem solved on its default grid under the
# solver options of `TestConfigs.ROBUSTNESS`. Reported, not tested (#360): which known
# solution each solve reached is in the report (`solution`); an exception still fails.
function test_robustness()
    C = Main.TestConfigs
    S = Main.SolveOptimalControl
    backend = OptimalControlProblems.OptimalControlBackend()
    Test.@testset "OptimalControl robustness report (default grid)" verbose = VERBOSE showtiming =
        SHOWTIMING begin
        for config in C.CONFIGS.OptimalControl,
            options in getfield(C.ROBUSTNESS, config.solver)

            label = C.label(merge(config, options))
            Test.@testset "$label" begin
                test = "test_robustness $label"
                Main.TestStatus.for_each_problem(test, backend) do problem
                    pb = getfield(OptimalControlProblems, problem)(backend)
                    N = OptimalControlProblems.grid_size_data(problem)
                    sol = nothing
                    time = @elapsed sol = S.solve(pb, config; grid_size=N, options=options)
                    Main.TestStatus.result!(
                        test,
                        problem,
                        backend,
                        label;
                        grid_size=N,
                        scheme=C.SCHEME,
                        S.outcome(sol)...,
                        time=time,
                    )
                    Test.@test true
                end
            end
        end
    end
end

end # module TestRobustnessOptimalControl

test_robustness() = TestRobustnessOptimalControl.test_robustness()
