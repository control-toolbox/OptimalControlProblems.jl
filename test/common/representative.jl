# The problems on which the generic parts of the models are tested against CTDirect: every
# scheme and ordering (`suite/comparison/test_equivalence`) and the solution built from a JuMP
# model (`suite/comparison/test_solution`), #441. The discretization is the same for every
# problem: testing it on all of them would test the same code again, so it is tested on a few
# problems chosen by their features (#376), the fewest that have every property the generic code
# treats differently. A problem is never named: the selection is computed from the catalogue.
module Representative

using OptimalControlProblems: OptimalControlProblems

const OCP = OptimalControlProblems

# Each property the generic code treats differently: the cost, the times, the dimensions of the
# state, the control and the variable (`DIM1`: one dimension is a scalar), the constraints.
const PROPERTIES = (
    mayer_cost=f -> f.cost === :mayer,
    lagrange_cost=f -> f.cost === :lagrange,
    bolza_cost=f -> f.cost === :bolza,
    free_initial_time=f -> f.initial_time === :free,
    free_final_time=f -> f.final_time === :free,
    scalar_state=f -> f.state_dimension == 1,
    vector_state=f -> f.state_dimension > 1,
    scalar_control=f -> f.control_dimension == 1,
    several_controls=f -> f.control_dimension > 1,
    no_variable=f -> f.variable_dimension == 0,
    scalar_variable=f -> f.variable_dimension == 1,
    vector_variable=f -> f.variable_dimension > 1,
    path_constraints=f -> f.constraints.path > 0,
    boundary_constraints=f -> f.constraints.boundary > 0,
    state_box=f -> f.constraints.state_box > 0,
    control_box=f -> f.constraints.control_box > 0,
)

"""The properties that occur among `problems`, whose features are `features`."""
function covered(problems, features)
    return [k for (k, has) in pairs(PROPERTIES) if any(p -> has(features[p]), problems)]
end

"""
The problems among `problems` that have, together, every property that occurs: each one is
the first that has the most properties still uncovered.
"""
function select(problems, features)
    uncovered = covered(problems, features)
    picked = Symbol[]
    while !isempty(uncovered)
        gain(p) = count(k -> PROPERTIES[k](features[p]), uncovered)
        best = argmax(gain, problems)
        gain(best) == 0 && break
        push!(picked, best)
        filter!(k -> !PROPERTIES[k](features[best]), uncovered)
    end
    return picked
end

"""
`(problems, features, selected)`: the problems available in the JuMP and the OptimalControl
modelings, their features, and the representative ones.
"""
function selection()
    problems = OCP.problems(OCP.JuMPBackend(), OCP.OptimalControlBackend())
    features = Dict(p => OCP.features(p) for p in problems)
    return (; problems, features, selected=select(problems, features))
end

end # module Representative
