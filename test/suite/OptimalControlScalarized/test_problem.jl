isdefined(Main, :OptimalControlTests) ||
    include(joinpath(@__DIR__, "..", "..", "common", "optimalcontrol_tests.jl"))

module TestProblemOptimalControlScalarized

using OptimalControlProblems: OptimalControlProblems

# The API of a problem of this modeling; written once in `common/optimalcontrol_tests.jl` (#441).
function test_problem()
    return Main.OptimalControlTests.test_problem(
        OptimalControlProblems.OptimalControlScalarizedBackend()
    )
end

end # module TestProblemOptimalControlScalarized

test_problem() = TestProblemOptimalControlScalarized.test_problem()
