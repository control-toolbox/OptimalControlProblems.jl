isdefined(Main, :SolveModelingToolkit) ||
    include(joinpath(@__DIR__, "..", "..", "common", "solve_modelingtoolkit.jl"))

module TestConvergenceModelingToolkit

using Test: Test
using ModelingToolkit: ModelingToolkit as MTK
using OptimalControlProblems: OptimalControlProblems

const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

function test_convergence()
    S = Main.SolveModelingToolkit
    backend = OptimalControlProblems.ModelingToolkitBackend()
    Test.@testset "ModelingToolkit convergence report (default grid)" verbose = VERBOSE showtiming =
        SHOWTIMING begin
        for config in Main.TestConfigs.CONFIGS.ModelingToolkit
            Test.@testset "$(Main.TestConfigs.label(config))" begin
                test = "test_convergence $(Main.TestConfigs.label(config))"
                Main.TestStatus.for_each_problem(test, backend) do problem
                    pb = getfield(OptimalControlProblems, problem)(backend)
                    N = OptimalControlProblems.grid_size_data(problem)
                    time = @elapsed sol = S.solve(
                        pb, config; grid_size=N, max_iter=Main.TestConfigs.MAX_ITER
                    )
                    scheme = Symbol(:shooting_, get(config, :shooting, :single))
                    stg = S.settings(config)
                    # convergence is reported, not tested (#360); an exception still fails
                    Main.TestStatus.result!(
                        test,
                        problem,
                        backend,
                        Main.TestConfigs.label(config);
                        modeler=stg.modeler,
                        solver=stg.solver,
                        options=stg.options,
                        grid_size=N,
                        scheme=scheme,
                        S.outcome(sol)...,
                        time=time,
                        trajectory=S.trajectory(sol, pb; grid_size=N),
                    )

                    # Log comparisons against analytic and trapezoidal reference objectives
                    analytic = OptimalControlProblems.analytic_reference(problem)
                    trapezoid = OptimalControlProblems.reference_objective(
                        problem; grid_size=N, scheme=:trapeze
                    )
                    if !isnothing(analytic)
                        gap_analytic =
                            abs(sol.objective - analytic.objective) / abs(analytic.objective)
                        println(
                            "  [$(problem)] gap to analytic objective ($(analytic.objective)): $(round(gap_analytic * 100; digits=4))%",
                        )
                    end
                    if !isnothing(trapezoid)
                        gap_trap = abs(sol.objective - trapezoid) / abs(trapezoid)
                        println(
                            "  [$(problem)] gap to OptimalControl trapezoid ($(trapezoid)): $(round(gap_trap * 100; digits=4))%",
                        )
                    end

                    Test.@test true
                end
            end
        end
    end
end

end # module TestConvergenceModelingToolkit

test_convergence() = TestConvergenceModelingToolkit.test_convergence()
