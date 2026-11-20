isdefined(Main, :JuMPTests) ||
    include(joinpath(@__DIR__, "..", "..", "common", "jump_tests.jl"))

module TestConvergenceJuMP

# The convergence report on the default grid of the modeling, written once in `common/modeling_tests.jl` (#449 T4).
test_convergence() = Main.JuMPTests.test_convergence()

end # module TestConvergenceJuMP

test_convergence() = TestConvergenceJuMP.test_convergence()
