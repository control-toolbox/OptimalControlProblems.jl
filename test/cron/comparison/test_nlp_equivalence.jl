isdefined(Main, :Equivalence) ||
    include(joinpath(@__DIR__, "..", "..", "common", "equivalence.jl"))

module TestNLPEquivalenceCron

using Test: Test
using OptimalControl: OptimalControl
using OptimalControlProblems: OptimalControlProblems

const OCP = OptimalControlProblems
const E = Main.Equivalence
const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

# Is the JuMP NLP of every problem CTDirect's? One configuration, the trapezoidal rule and the
# interleaved ordering, for every problem (#441): the discretization of CTDirect is compiled for
# each problem, which a pull request does not afford; the cron does. Unlike the convergence
# reports, this is a test: no solver is run, the NLPs are built and evaluated, so a difference
# does not depend on the platform and fails the run. It tests the JuMP discretization, which this
# package writes: it goes when it moves to a package of its own.
function test_nlp_equivalence()
    Test.@testset "NLP equivalence with CTDirect, every problem" verbose = VERBOSE showtiming =
        SHOWTIMING begin
        Main.TestStatus.for_each_problem(
            "NLP equivalence, every problem", OCP.JuMPBackend(), OCP.OptimalControlBackend()
        ) do problem
            E.test_problem(problem, (:trapeze,), (:interleaved,))
        end
    end
end

end # module TestNLPEquivalenceCron

test_nlp_equivalence() = TestNLPEquivalenceCron.test_nlp_equivalence()
