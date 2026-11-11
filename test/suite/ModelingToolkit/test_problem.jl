isdefined(Main, :ModelingToolkitTests) ||
    include(joinpath(@__DIR__, "..", "..", "common", "modelingtoolkit_tests.jl"))

module TestProblemModelingToolkit

using OptimalControlProblems: OptimalControlProblems

# Written once in `common/modeling_tests.jl` (#441, #439).
test_problem() = Main.ModelingToolkitTests.test_problem(OptimalControlProblems.ModelingToolkitBackend())

end # module TestProblemModelingToolkit

test_problem() = TestProblemModelingToolkit.test_problem()
