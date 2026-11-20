isdefined(Main, :ModelingToolkitTests) ||
    include(joinpath(@__DIR__, "..", "..", "common", "modelingtoolkit_tests.jl"))

module TestDraftModelingToolkit

using OptimalControlProblems: OptimalControlProblems

# A draft warns when it is built, a valid problem does not; written once in
# `common/modeling_tests.jl` (#449 T4).
function test_draft()
    return Main.ModelingToolkitTests.test_draft(
        OptimalControlProblems.ModelingToolkitBackend()
    )
end

end # module TestDraftModelingToolkit

test_draft() = TestDraftModelingToolkit.test_draft()
