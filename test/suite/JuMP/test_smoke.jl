isdefined(Main, :JuMPTests) ||
    include(joinpath(@__DIR__, "..", "..", "common", "jump_tests.jl"))

module TestSmokeJuMP

# Every problem builds, evaluates and runs a few iterations, written once in `common/modeling_tests.jl` (#449 T4).
test_smoke() = Main.JuMPTests.test_smoke()

end # module TestSmokeJuMP

test_smoke() = TestSmokeJuMP.test_smoke()
