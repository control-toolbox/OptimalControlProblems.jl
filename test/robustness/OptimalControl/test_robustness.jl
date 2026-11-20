isdefined(Main, :OptimalControlTests) ||
    include(joinpath(@__DIR__, "..", "..", "common", "optimalcontrol_tests.jl"))

module TestRobustnessOptimalControl

using OptimalControlProblems: OptimalControlProblems

# The robustness report of the initial guess (#378); written once in
# `common/modeling_tests.jl` (#449 T4).
function test_robustness()
    return Main.OptimalControlTests.test_robustness(
        OptimalControlProblems.OptimalControlBackend()
    )
end

end # module TestRobustnessOptimalControl

test_robustness() = TestRobustnessOptimalControl.test_robustness()
