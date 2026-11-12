isdefined(Main, :ModelingToolkitTests) ||
    include(joinpath(@__DIR__, "..", "..", "common", "modelingtoolkit_tests.jl"))

module TestSmokeModelingToolkit

using OptimalControlProblems: OptimalControlProblems

# Written once in `common/modeling_tests.jl` (#441, #439).
function test_smoke()
    return Main.ModelingToolkitTests.test_smoke(
        OptimalControlProblems.ModelingToolkitBackend()
    )
end

end # module TestSmokeModelingToolkit

test_smoke() = TestSmokeModelingToolkit.test_smoke()
