isdefined(Main, :OptimalControlTests) ||
    include(joinpath(@__DIR__, "..", "..", "common", "optimalcontrol_tests.jl"))

module TestDraftOptimalControlScalarized

using OptimalControlProblems: OptimalControlProblems

# A draft warns when it is built, a valid problem does not; written once in
# `common/modeling_tests.jl` (#449 T4).
function test_draft()
    return Main.OptimalControlTests.test_draft(
        OptimalControlProblems.OptimalControlScalarizedBackend()
    )
end

end # module TestDraftOptimalControlScalarized

test_draft() = TestDraftOptimalControlScalarized.test_draft()
