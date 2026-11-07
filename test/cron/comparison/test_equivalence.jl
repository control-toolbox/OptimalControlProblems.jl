isdefined(Main, :Equivalence) ||
    include(joinpath(@__DIR__, "..", "..", "common", "equivalence.jl"))

module TestEquivalenceCron

using Test: Test
using OptimalControlProblems: OptimalControlProblems

const OCP = OptimalControlProblems
const E = Main.Equivalence
const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

# Every problem in both modelings, every scheme and ordering (#371).
function test_equivalence()
    Test.@testset "NLP equivalence with CTDirect, every scheme" verbose = VERBOSE showtiming =
        SHOWTIMING begin
        Main.TestStatus.for_each_problem(
            "NLP equivalence (every scheme)", OCP.JuMPBackend(), OCP.OptimalControlBackend()
        ) do problem
            E.test_problem(problem, E.SCHEMES)
        end
        Main.TestStatus.for_each_problem(
            "NLP equivalence, scalarized (every scheme)",
            OCP.OptimalControlBackend(),
            OCP.OptimalControlScalarizedBackend(),
        ) do problem
            E.test_scalarized(problem, E.SCHEMES)
        end
    end
end

end # module TestEquivalenceCron

test_equivalence() = TestEquivalenceCron.test_equivalence()
