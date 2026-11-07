isdefined(Main, :Equivalence) ||
    include(joinpath(@__DIR__, "..", "..", "common", "equivalence.jl"))

module TestEquivalence

using Test: Test
using OptimalControl: OptimalControl
using OptimalControlProblems: OptimalControlProblems

const OCP = OptimalControlProblems
const E = Main.Equivalence
const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

# Fast selection (the full matrix is in `cron/comparison/`, about 20 minutes): the
# trapezoidal rule for every problem, the four schemes for two problems chosen by their
# properties, the first one with nonlinear path constraints and the first one with a
# variable (e.g. a free final time).
function every_scheme()
    both = OCP.problems(OCP.JuMPBackend(), OCP.OptimalControlBackend())
    ocp(p) = OCP.model(getfield(OCP, p)(OCP.OptimalControlBackend()))
    with_path = findfirst(p -> OptimalControl.dim_path_constraints_nl(ocp(p)) > 0, both)
    with_variable = findfirst(p -> OptimalControl.variable_dimension(ocp(p)) > 0, both)
    return unique(both[[with_path, with_variable]])
end

function test_equivalence()
    Test.@testset "NLP equivalence with CTDirect" verbose = VERBOSE showtiming = SHOWTIMING begin
        selected = every_scheme()
        Test.@test length(selected) == 2
        Main.TestStatus.for_each_problem(
            "NLP equivalence", OCP.JuMPBackend(), OCP.OptimalControlBackend()
        ) do problem
            E.test_problem(problem, problem in selected ? E.SCHEMES : (:trapeze,))
        end
        E.test_nonzeros()
    end
end

end # module TestEquivalence

test_equivalence() = TestEquivalence.test_equivalence()
