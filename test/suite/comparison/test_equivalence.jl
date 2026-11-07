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

# The trapezoidal rule for every problem: are two modelings the same problem? One scheme
# shows it. Every scheme and ordering on a few problems: are the JuMP schemes CTDirect's?
# A question on `schemes/build.jl`, not on the problems (#401), also answered by the unit
# tests of `suite/JuMP/test_build.jl`. The problems are chosen by their features (#376):
# for each property that the schemes treat differently, the first problem that has it.
const PROPERTIES = (
    mayer_cost=f -> f.cost === :mayer,
    lagrange_cost=f -> f.cost === :lagrange,
    bolza_cost=f -> f.cost === :bolza,
    free_initial_time=f -> f.initial_time === :free,
    free_final_time=f -> f.final_time === :free,
    variable=f -> f.variable_dimension > 0,
    several_controls=f -> f.control_dimension > 1,
    path_constraints=f -> f.constraints.path > 0,
    boundary_constraints=f -> f.constraints.boundary > 0,
    state_box=f -> f.constraints.state_box > 0,
    control_box=f -> f.constraints.control_box > 0,
)

"""The properties that occur among `problems`, whose features are `features`."""
function covered(problems, features)
    return [k for (k, has) in pairs(PROPERTIES) if any(p -> has(features[p]), problems)]
end

"""The problems on which every scheme is compared: for each property, the first one."""
function every_scheme(problems, features)
    picks = [findfirst(p -> has(features[p]), problems) for has in PROPERTIES]
    return unique(problems[filter(!isnothing, picks)])
end

function test_equivalence()
    Test.@testset "NLP equivalence with CTDirect" verbose = VERBOSE showtiming = SHOWTIMING begin
        both = OCP.problems(OCP.JuMPBackend(), OCP.OptimalControlBackend())
        features = Dict(p => OCP.features(p) for p in both)
        selected = every_scheme(both, features)
        @info "Every scheme and ordering on: $(join(selected, ", "))"
        Test.@test covered(selected, features) == covered(both, features)
        Main.TestStatus.for_each_problem(
            "NLP equivalence", OCP.JuMPBackend(), OCP.OptimalControlBackend()
        ) do problem
            E.test_problem(problem, problem in selected ? E.SCHEMES : (:trapeze,))
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
