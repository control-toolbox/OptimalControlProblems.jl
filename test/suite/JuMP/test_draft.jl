isdefined(Main, :JuMPTests) ||
    include(joinpath(@__DIR__, "..", "..", "common", "jump_tests.jl"))

module TestDraftJuMP

# A draft warns when it is built, a valid problem does not, written once in `common/modeling_tests.jl` (#449 T4).
test_draft() = Main.JuMPTests.test_draft()

end # module TestDraftJuMP

test_draft() = TestDraftJuMP.test_draft()
