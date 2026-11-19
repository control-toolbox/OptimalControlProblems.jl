isdefined(Main, :ModelingToolkitTests) ||
    include(joinpath(@__DIR__, "..", "..", "common", "modelingtoolkit_tests.jl"))

module TestProblemModelingToolkit

using OptimalControlProblems: OptimalControlProblems

# Written once in `common/modeling_tests.jl` (#441, #439).
function test_problem()
    return Main.ModelingToolkitTests.test_problem(
        OptimalControlProblems.ModelingToolkitBackend()
    )
end

end # module TestProblemModelingToolkit

test_problem() = TestProblemModelingToolkit.test_problem()
