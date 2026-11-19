isdefined(Main, :ModelingToolkitTests) ||
    include(joinpath(@__DIR__, "..", "..", "common", "modelingtoolkit_tests.jl"))

module TestConvergenceModelingToolkit

using OptimalControlProblems: OptimalControlProblems

# The convergence report on the default grid; written once in `common/modeling_tests.jl`
# (#441, #439). The gap is to the objective of the analytic solution, when there is one.
function test_convergence()
    return Main.ModelingToolkitTests.test_convergence(
        OptimalControlProblems.ModelingToolkitBackend()
    )
end

end # module TestConvergenceModelingToolkit

test_convergence() = TestConvergenceModelingToolkit.test_convergence()
