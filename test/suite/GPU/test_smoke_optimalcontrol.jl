isdefined(Main, :GPUTests) ||
    include(joinpath(@__DIR__, "..", "..", "common", "gpu_tests.jl"))

module TestSmokeGPUOptimalControl

using OptimalControlProblems: OptimalControlProblems

# Exa and MadNLP on the GPU; written once in `common/gpu_tests.jl` (#449 T4).
function test_smoke_optimalcontrol()
    return Main.GPUTests.test_smoke(OptimalControlProblems.OptimalControlBackend())
end

end # module TestSmokeGPUOptimalControl

test_smoke_optimalcontrol() = TestSmokeGPUOptimalControl.test_smoke_optimalcontrol()
