isdefined(Main, :TestCapabilities) ||
    include(joinpath(@__DIR__, "..", "..", "common", "capabilities.jl"))
isdefined(Main, :SolveOptimalControl) ||
    include(joinpath(@__DIR__, "..", "..", "common", "solve_optimalcontrol.jl"))

module TestSolveGPUOptimalControl

using Test: Test
using CUDA: CUDA
using CUDSS: CUDSS
using MadNLPGPU: MadNLPGPU
using OptimalControl: OptimalControl # loads the extensions of the modeling and of the GPU
using OptimalControlProblems: OptimalControlProblems

const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

# Exa and MadNLP on the GPU, small grid: every problem of the modeling must solve.
function test_solve_optimalcontrol()
    S = Main.SolveOptimalControl
    backend = OptimalControlProblems.OptimalControlBackend()
    Test.@testset "OptimalControl on GPU (small grid)" verbose = VERBOSE showtiming = SHOWTIMING begin
        for config in Main.TestConfigs.GPU_CONFIGS.OptimalControl
            Test.@testset "$(Main.TestConfigs.label(config))" begin
                test = "test_solve_optimalcontrol $(Main.TestConfigs.label(config))"
                Main.TestStatus.for_each_problem(test, backend) do problem
                    pb = getfield(OptimalControlProblems, problem)(backend)
                    N = Main.TestConfigs.SMALL_GRID
                    if Main.TestCapabilities.CUDA_FUNCTIONAL
                        Test.@test S.succeeded(S.solve_gpu(pb, config; grid_size=N))
                    else
                        # no device: a visible skip (TST6), not a vacuous pass
                        Test.@test_skip S.succeeded(S.solve_gpu(pb, config; grid_size=N))
                    end
                end
            end
        end
    end
end

end # module TestSolveGPUOptimalControl

test_solve_optimalcontrol() = TestSolveGPUOptimalControl.test_solve_optimalcontrol()
