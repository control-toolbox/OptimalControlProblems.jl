isdefined(Main, :OptimalControlTests) ||
    include(joinpath(@__DIR__, "..", "..", "common", "optimalcontrol_tests.jl"))

module TestConvergenceOptimalControl

using OptimalControlProblems: OptimalControlProblems

# The convergence report on the default grid; written once in `common/optimalcontrol_tests.jl`
# (#441).
test_convergence() = Main.OptimalControlTests.test_convergence(
    OptimalControlProblems.OptimalControlBackend()
)

end # module TestConvergenceOptimalControl

test_convergence() = TestConvergenceOptimalControl.test_convergence()
