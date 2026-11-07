isdefined(Main, :SolveOptimalControl) ||
    include(joinpath(@__DIR__, "..", "..", "common", "solve_optimalcontrol.jl"))

module TestConvergenceOptimalControlScalarized

using Test: Test
using OptimalControl: OptimalControl # loads the OptimalControlModels extension
using OptimalControlProblems: OptimalControlProblems

const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

function test_convergence()
    S = Main.SolveOptimalControl
    backend = OptimalControlProblems.OptimalControlScalarizedBackend()
    Test.@testset "OptimalControlScalarized convergence report (default grid)" verbose = VERBOSE showtiming =
        SHOWTIMING begin
        for config in Main.TestConfigs.CONFIGS.OptimalControlScalarized
            Test.@testset "$(Main.TestConfigs.label(config))" begin
                test = "test_convergence $(Main.TestConfigs.label(config))"
                Main.TestStatus.for_each_problem(test, backend) do problem
                    pb = getfield(OptimalControlProblems, problem)(backend)
                    N = OptimalControlProblems.grid_size_data(problem)
                    sol = S.solve(pb, config; grid_size=N)
                    time = @elapsed sol = S.solve(pb, config; grid_size=N) # second, compiled
                    # convergence is reported, not tested (#360); an exception still fails
                    Main.TestStatus.result!(
                        test, problem, backend, Main.TestConfigs.label(config);
                        S.settings(config)...,
                        grid_size=N, scheme=Main.TestConfigs.SCHEME, S.outcome(sol)...,
                        time=time, trajectory=Main.TestStatus.trajectory(sol),
                    )
                    Test.@test true
                end
            end
        end
    end
end

end # module TestConvergenceOptimalControlScalarized

test_convergence() = TestConvergenceOptimalControlScalarized.test_convergence()
