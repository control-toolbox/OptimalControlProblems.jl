isdefined(Main, :JuMPTests) ||
    include(joinpath(@__DIR__, "..", "..", "common", "jump_tests.jl"))

module TestRobustnessJuMP

# The robustness report of the initial guess (#378), written once in `common/modeling_tests.jl` (#449 T4).
test_robustness() = Main.JuMPTests.test_robustness()

end # module TestRobustnessJuMP

test_robustness() = TestRobustnessJuMP.test_robustness()
