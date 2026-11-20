isdefined(Main, :GPUTests) ||
    include(joinpath(@__DIR__, "..", "..", "common", "gpu_tests.jl"))

module TestSmokeGPUOptimalControlScalarized

using OptimalControlProblems: OptimalControlProblems

# Exa and MadNLP on the GPU; written once in `common/gpu_tests.jl` (#449 T4).
function test_smoke_scalarized()
    return Main.GPUTests.test_smoke(
        OptimalControlProblems.OptimalControlScalarizedBackend()
    )
end

end # module TestSmokeGPUOptimalControlScalarized

test_smoke_scalarized() = TestSmokeGPUOptimalControlScalarized.test_smoke_scalarized()
