isdefined(Main, :TestCapabilities) ||
    include(joinpath(@__DIR__, "..", "..", "common", "capabilities.jl"))
isdefined(Main, :SolveOptimalControl) ||
    include(joinpath(@__DIR__, "..", "..", "common", "solve_optimalcontrol.jl"))

module TestSmokeGPUOptimalControlScalarized

using Test: Test
using CUDA: CUDA
using CUDSS: CUDSS
using MadNLPGPU: MadNLPGPU
using OptimalControl: OptimalControl # loads the extensions of the modeling and of the GPU
using OptimalControlProblems: OptimalControlProblems

const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

# Exa and MadNLP on the GPU, small grid: every problem of the modeling must solve.
function test_smoke_scalarized()
    S = Main.SolveOptimalControl
    backend = OptimalControlProblems.OptimalControlScalarizedBackend()
    Test.@testset "OptimalControlScalarized smoke on GPU (small grid, few iterations)" verbose =
        VERBOSE showtiming = SHOWTIMING begin
        for config in Main.TestConfigs.GPU_CONFIGS.OptimalControlScalarized
            Test.@testset "$(Main.TestConfigs.label(config))" begin
                test = "test_smoke_scalarized $(Main.TestConfigs.label(config))"
                Main.TestStatus.for_each_problem(test, backend) do problem
                    pb = getfield(OptimalControlProblems, problem)(backend)
                    o = Main.TestConfigs
                    if Main.TestCapabilities.CUDA_FUNCTIONAL
                        # a few iterations: the solve runs on the device and gives a finite
                        # objective; whether it converges is not checked (#360)
                        sol = S.solve(
                            pb, config; grid_size=o.SMOKE_GRID, max_iter=o.SMOKE_MAX_ITER
                        )
                        Test.@test isfinite(OptimalControl.objective(sol))
                    else
                        # no device: a visible skip (TST6), not a vacuous pass
                        Test.@test_skip isfinite(
                            OptimalControl.objective(
                                S.solve(
                                    pb,
                                    config;
                                    grid_size=o.SMOKE_GRID,
                                    max_iter=o.SMOKE_MAX_ITER,
                                ),
                            ),
                        )
                    end
                end
            end
        end
    end
end

end # module TestSmokeGPUOptimalControlScalarized

test_smoke_scalarized() = TestSmokeGPUOptimalControlScalarized.test_smoke_scalarized()
