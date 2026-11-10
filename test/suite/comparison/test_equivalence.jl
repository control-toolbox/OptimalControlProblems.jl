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

# The trapezoidal rule and the interleaved ordering for every problem: are two modelings the
# same problem? One configuration shows it. Every scheme and ordering on a few problems: are the
# JuMP schemes CTDirect's? A question on `schemes/build.jl`, not on the problems (#401, #441),
# also answered by the unit tests of `suite/JuMP/test_build.jl`. The problems are chosen by their
# features (#376), see `common/representative.jl`.
function test_equivalence()
    Test.@testset "NLP equivalence with CTDirect" verbose = VERBOSE showtiming = SHOWTIMING begin
        R = Main.Representative
        both, features, selected = R.selection()
        @info "Every scheme and ordering on: $(join(selected, ", "))"
        Test.@test R.covered(selected, features) == R.covered(both, features)
        Main.TestStatus.for_each_problem(
            "NLP equivalence", OCP.JuMPBackend(), OCP.OptimalControlBackend()
        ) do problem
            every = problem in selected
            E.test_problem(
                problem, every ? E.SCHEMES : (:trapeze,), every ? E.ORDERINGS : (:interleaved,)
            )
        end
        E.test_nonzeros()
        Main.TestStatus.for_each_problem(
            "NLP equivalence, scalarized",
            OCP.OptimalControlBackend(),
            OCP.OptimalControlScalarizedBackend(),
        ) do problem
            E.test_scalarized(problem)
        end
    end
end

end # module TestEquivalence

test_equivalence() = TestEquivalence.test_equivalence()
