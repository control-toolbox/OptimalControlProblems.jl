# ------- Problem: the continuous problem returned by the OptimalControl modelings -------

"""
$(TYPEDEF)

An optimal control problem of the collection: the continuous model, as written with
OptimalControl, and its initial guess. Discretizing and solving it is left to
`OptimalControl.solve`, where the scheme, the grid, the modeler and the solver are chosen.

A `Problem` is returned by a problem function for the OptimalControl modelings. Read it
through `model(pb)`, [`initial_guess`](@ref) and `metadata(pb)`.

# Fields

$(TYPEDFIELDS)

# Example

```julia-repl
julia> using OptimalControl, OptimalControlProblems
julia> pb = beam(OptimalControlBackend())
julia> sol = solve(model(pb); init=initial_guess(pb), grid_size=500)
```
"""
struct Problem{M,I<:NamedTuple}
    "Name of the problem."
    name::Symbol
    "Modeling the problem is written in, e.g. `:OptimalControl`."
    modeling::Symbol
    "Continuous optimal control problem."
    ocp::M
    "Initial guess, a `NamedTuple` `(state=…, control=…, variable=…)`."
    init::I
    "Metadata of the problem, with the parameters this instance was built with."
    metadata::OrderedDict{Symbol,Any}
end

# Called at the end of every problem file of the OptimalControl modelings.
function _problem(
    problem::Symbol,
    modeling::Symbol,
    ocp,
    init::NamedTuple,
    parameters::Union{Nothing,NamedTuple},
)
    data = copy(problem_metadata(problem))
    data[:parameters] = parameters
    return Problem(problem, modeling, ocp, init, data)
end

"""
$(TYPEDSIGNATURES)

Return the continuous optimal control problem, to be given to `OptimalControl.solve` or
`OptimalControl.discretize`, as `model(sol)` returns the problem of a solution.
"""
CTModels.Solutions.model(pb::Problem) = pb.ocp

"""
$(TYPEDSIGNATURES)

Return the initial guess of the problem, a `NamedTuple` `(state=…, control=…, variable=…)`
to be given as `init` to `OptimalControl.solve`.
"""
initial_guess(pb::Problem) = pb.init

"""
$(TYPEDSIGNATURES)

Return the metadata of the problem: default grid size, the parameters it was built with,
and its status in each modeling. See [`problem_metadata`](@ref) for the keys.
"""
CTBase.Strategies.metadata(pb::Problem) = pb.metadata

function Base.show(io::IO, pb::Problem)
    return print(io, "Problem(:", pb.name, ")")
end

function Base.show(io::IO, ::MIME"text/plain", pb::Problem)
    model = pb.ocp
    status = get(pb.metadata[:status], pb.modeling, :unknown)
    println(io, "Problem ", pb.name, " (", pb.modeling, " modeling, ", status, ")")
    print(io, "  state dimension:    ", CTModels.Models.state_dimension(model), "\n")
    print(io, "  control dimension:  ", CTModels.Models.control_dimension(model), "\n")
    print(io, "  variable dimension: ", CTModels.Models.variable_dimension(model))
    return nothing
end
