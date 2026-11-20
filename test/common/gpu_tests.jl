# The GPU smoke solves of the OptimalControl modelings (environment `GPU`), written once and
# parameterized by the backend (#449 T4): Exa and MadNLP on the device, small grid, a few
# iterations (`Main.TestConfigs.GPU_CONFIGS`). Without a functional device the solves are a
# visible skip (TST6), not a vacuous pass. A test file of the environment is a wrapper.
isdefined(Main, :TestCapabilities) || include(joinpath(@__DIR__, "capabilities.jl"))
isdefined(Main, :SolveOptimalControl) ||
    include(joinpath(@__DIR__, "solve_optimalcontrol.jl"))

module GPUTests

using Test: Test
using CUDA: CUDA
using CUDSS: CUDSS
using MadNLPGPU: MadNLPGPU
using OptimalControl: OptimalControl # loads the extensions of the modeling and of the GPU
using OptimalControlProblems: OptimalControlProblems

const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

"""Every problem of the modeling of `backend` runs a few iterations on the device."""
function test_smoke(backend)
    S = Main.SolveOptimalControl
    o = Main.TestConfigs
    name = Main.TestStatus.modeling(backend)
    Test.@testset "$name smoke on GPU (small grid, few iterations)" verbose = VERBOSE showtiming =
        SHOWTIMING begin
        for config in getfield(o.GPU_CONFIGS, name)
            Test.@testset "$(o.label(config))" begin
                test = "test_smoke_gpu $(o.label(config))"
                Main.TestStatus.for_each_problem(test, backend) do problem
                    pb = getfield(OptimalControlProblems, problem)(backend)
                    solve() = S.solve(
                        pb, config; grid_size=o.SMOKE_GRID, max_iter=o.SMOKE_MAX_ITER
                    )
                    if Main.TestCapabilities.CUDA_FUNCTIONAL
                        # whether it converges is not checked (#360)
                        Test.@test isfinite(OptimalControl.objective(solve()))
                    else
                        Test.@test_skip isfinite(OptimalControl.objective(solve()))
                    end
                end
            end
        end
    end
end

end # module GPUTests
