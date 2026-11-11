isdefined(Main, :OptimalControlTests) ||
    include(joinpath(@__DIR__, "..", "..", "common", "optimalcontrol_tests.jl"))

module TestProblemOptimalControl

using OptimalControlProblems: OptimalControlProblems

# The API of a problem of this modeling; written once in `common/optimalcontrol_tests.jl` (#441).
test_problem() = Main.OptimalControlTests.test_problem(
    OptimalControlProblems.OptimalControlBackend()
)

end # module TestProblemOptimalControl

test_problem() = TestProblemOptimalControl.test_problem()
