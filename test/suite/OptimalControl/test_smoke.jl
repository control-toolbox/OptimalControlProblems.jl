isdefined(Main, :OptimalControlTests) ||
    include(joinpath(@__DIR__, "..", "..", "common", "optimalcontrol_tests.jl"))

module TestSmokeOptimalControl

using OptimalControlProblems: OptimalControlProblems

# Every problem builds, evaluates and runs a few iterations in each configuration; written once
# in `common/optimalcontrol_tests.jl` (#441).
test_smoke() = Main.OptimalControlTests.test_smoke(
    OptimalControlProblems.OptimalControlBackend()
)

end # module TestSmokeOptimalControl

test_smoke() = TestSmokeOptimalControl.test_smoke()
