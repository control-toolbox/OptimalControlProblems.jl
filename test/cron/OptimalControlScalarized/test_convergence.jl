isdefined(Main, :OptimalControlTests) ||
    include(joinpath(@__DIR__, "..", "..", "common", "optimalcontrol_tests.jl"))

module TestConvergenceOptimalControlScalarized

using OptimalControlProblems: OptimalControlProblems

# The convergence report on the default grid; written once in `common/optimalcontrol_tests.jl`
# (#441).
function test_convergence()
    return Main.OptimalControlTests.test_convergence(
        OptimalControlProblems.OptimalControlScalarizedBackend()
    )
end

end # module TestConvergenceOptimalControlScalarized

test_convergence() = TestConvergenceOptimalControlScalarized.test_convergence()
