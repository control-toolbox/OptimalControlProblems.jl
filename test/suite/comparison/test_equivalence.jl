isdefined(Main, :Equivalence) ||
    include(joinpath(@__DIR__, "..", "..", "common", "equivalence.jl"))
isdefined(Main, :Representative) ||
    include(joinpath(@__DIR__, "..", "..", "common", "representative.jl"))

module TestEquivalence

using Test: Test
using OptimalControl: OptimalControl
using OptimalControlProblems: OptimalControlProblems

const OCP = OptimalControlProblems
const E = Main.Equivalence
const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

# Is the discretization of the JuMP models CTDirect's? Every scheme and ordering, on a few
# problems chosen by their features (see `common/representative.jl`): the discretization is the
# same code for every problem (#401, #441). The modelings are compared on every problem by
# `test_modelings`, without discretizing; the NLP of every problem is compared in the cron
# (`cron/comparison/`). This tests the JuMP discretization, which this package writes: it goes
# when it moves to a package of its own.
function test_equivalence()
    Test.@testset "NLP equivalence with CTDirect" verbose = VERBOSE showtiming = SHOWTIMING begin
        R = Main.Representative
        both, features, selected = R.selection()
        @info "Every scheme and ordering on: $(join(selected, ", "))"
        Test.@test R.covered(selected, features) == R.covered(both, features)
        Main.TestStatus.for_each_problem(
            "NLP equivalence", OCP.JuMPBackend(), OCP.OptimalControlBackend(); only=selected
        ) do problem
            E.test_problem(problem, E.SCHEMES, E.ORDERINGS)
        end
    end
end

end # module TestEquivalence

test_equivalence() = TestEquivalence.test_equivalence()
