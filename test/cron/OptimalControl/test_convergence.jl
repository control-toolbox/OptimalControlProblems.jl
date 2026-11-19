isdefined(Main, :OptimalControlTests) ||
    include(joinpath(@__DIR__, "..", "..", "common", "optimalcontrol_tests.jl"))

module TestConvergenceOptimalControl

using OptimalControlProblems: OptimalControlProblems

# The convergence report on the default grid; written once in `common/optimalcontrol_tests.jl`
# (#441).
function test_convergence()
    return Main.OptimalControlTests.test_convergence(
        OptimalControlProblems.OptimalControlBackend()
    )
end

end # module TestConvergenceOptimalControl

test_convergence() = TestConvergenceOptimalControl.test_convergence()
