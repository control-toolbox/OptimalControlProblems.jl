isdefined(Main, :OptimalControlTests) ||
    include(joinpath(@__DIR__, "..", "..", "common", "optimalcontrol_tests.jl"))

module TestSmokeOptimalControlScalarized

using OptimalControlProblems: OptimalControlProblems

# Every problem builds, evaluates and runs a few iterations in each configuration; written once
# in `common/optimalcontrol_tests.jl` (#441).
function test_smoke()
    return Main.OptimalControlTests.test_smoke(
        OptimalControlProblems.OptimalControlScalarizedBackend()
    )
end

end # module TestSmokeOptimalControlScalarized

test_smoke() = TestSmokeOptimalControlScalarized.test_smoke()
