isdefined(Main, :OptimalControlTests) ||
    include(joinpath(@__DIR__, "..", "..", "common", "optimalcontrol_tests.jl"))

module TestDraftOptimalControl

using OptimalControlProblems: OptimalControlProblems

# A draft warns when it is built, a valid problem does not; written once in
# `common/modeling_tests.jl` (#449 T4).
function test_draft()
    return Main.OptimalControlTests.test_draft(
        OptimalControlProblems.OptimalControlBackend()
    )
end

end # module TestDraftOptimalControl

test_draft() = TestDraftOptimalControl.test_draft()
