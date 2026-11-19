module ModelingToolkitModels

using OptimalControlProblems: OptimalControlProblems, ModelingToolkitBackend
using ModelingToolkit: ModelingToolkit
using CTModels: CTModels
using DocStringExtensions: TYPEDSIGNATURES

# Include only files belonging to this modeling. The file presence is the
# source of truth for availability.
rel_path_problems = "ModelingToolkitModels"
for problem in OptimalControlProblems.problems(ModelingToolkitBackend(); status=:all)
    include(joinpath(rel_path_problems, "$(problem).jl"))
end

# Strip '(t)' or '(..)' parameter/call syntax from symbolic names to match standard component names
_strip_symbolic_call(s::AbstractString) = replace(s, r"\(.*\)$" => "")

"""
$(TYPEDSIGNATURES)

Return the state component names of a `ModelingToolkit.AbstractSystem`.
"""
function CTModels.Models.state_components(sys::ModelingToolkit.AbstractSystem)
    states = filter(!ModelingToolkit.isinput, ModelingToolkit.unknowns(sys))
    return [_strip_symbolic_call(string(s)) for s in states]
end

"""
$(TYPEDSIGNATURES)

Return the number of states of a `ModelingToolkit.AbstractSystem`.
"""
function CTModels.Models.state_dimension(sys::ModelingToolkit.AbstractSystem)
    return length(CTModels.Models.state_components(sys))
end

"""
$(TYPEDSIGNATURES)

Return the control component names of a `ModelingToolkit.AbstractSystem`.
"""
function CTModels.Models.control_components(sys::ModelingToolkit.AbstractSystem)
    controls = filter(ModelingToolkit.isinput, ModelingToolkit.unknowns(sys))
    return [_strip_symbolic_call(string(c)) for c in controls]
end

"""
$(TYPEDSIGNATURES)

Return the number of controls of a `ModelingToolkit.AbstractSystem`.
"""
function CTModels.Models.control_dimension(sys::ModelingToolkit.AbstractSystem)
    return length(CTModels.Models.control_components(sys))
end

"""
$(TYPEDSIGNATURES)

Return the variable component names of a `ModelingToolkit.AbstractSystem`.
"""
function CTModels.Models.variable_components(sys::ModelingToolkit.AbstractSystem)
    vars = filter(
        p -> ModelingToolkit.istunable(p, true) && !ModelingToolkit.isinput(p),
        ModelingToolkit.parameters(sys),
    )
    return [_strip_symbolic_call(string(v)) for v in vars]
end

"""
$(TYPEDSIGNATURES)

Return the number of variables of a `ModelingToolkit.AbstractSystem`.
"""
function CTModels.Models.variable_dimension(sys::ModelingToolkit.AbstractSystem)
    return length(CTModels.Models.variable_components(sys))
end

end # module ModelingToolkitModels
