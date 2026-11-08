# ------- Problem: the continuous problem returned by the OptimalControl modelings -------

"""
$(TYPEDEF)

An optimal control problem of the collection: the continuous model, as written with
OptimalControl, and its initial guess. Discretizing and solving it is left to
`OptimalControl.solve`, where the scheme, the grid, the modeler and the solver are chosen.

A `Problem` is returned by a problem function for the OptimalControl modelings. Read it
through `model(pb)`, [`initial_guess`](@ref OptimalControlProblems.Problems.initial_guess) and `metadata(pb)`.

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
    data = copy(Metadata.problem_metadata(problem))
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
and its status in each modeling. See [`problem_metadata`](@ref OptimalControlProblems.Metadata.problem_metadata) for the keys.
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

"""
    solution(model, pb::Problem) -> CTModels.Solution

Build the `CTModels.Solution` of a solved model of the same problem as `pb`, to show and plot
it as an OptimalControl solution: `state(sol)`, `control(sol)`, `costate(sol)`,
`objective(sol)`, `iterations(sol)` and `plot(sol)` work on it, and `model(sol) === model(pb)`.

The method for a JuMP model is defined by the `JuMPModels` extension:
load JuMP. The model and `pb` must have the same state, control and variable dimensions.
"""
function solution end

"""
$(TYPEDSIGNATURES)

Features of `problem`, computed from its OptimalControl model, never written by hand:

- `state_dimension`, `control_dimension`, `variable_dimension`;
- `cost`: `:mayer`, `:lagrange` or `:bolza`;
- `initial_time`, `final_time`: `:fixed` or `:free`;
- `constraints`: the number of constraints of each kind, `state_box`, `control_box`,
  `variable_box` (box constraints), `path` (nonlinear path constraints) and `boundary`
  (nonlinear boundary constraints).

The problem is built with its default parameters, which needs the `OptimalControlModels`
extension: load OptimalControl. A problem without an OptimalControl model has no features
yet.

# Example

```julia-repl
julia> using OptimalControl, OptimalControlProblems

julia> OptimalControlProblems.features(:beam).cost
:lagrange
```
"""
function features(problem::Symbol)
    return features(getfield(Catalogue, problem)(Modelings.OptimalControlBackend()))
end

"""
$(TYPEDSIGNATURES)

Features of the `Problem` `pb`, computed from its model (see `features(problem::Symbol)`).
"""
function features(pb::Problem)
    ocp = model(pb)
    M = CTModels.Models
    mayer, lagrange = M.has_mayer_cost(ocp), M.has_lagrange_cost(ocp)
    return (
        state_dimension=M.state_dimension(ocp),
        control_dimension=M.control_dimension(ocp),
        variable_dimension=M.variable_dimension(ocp),
        cost=mayer && lagrange ? :bolza : (mayer ? :mayer : :lagrange),
        initial_time=M.has_fixed_initial_time(ocp) ? :fixed : :free,
        final_time=M.has_fixed_final_time(ocp) ? :fixed : :free,
        constraints=(
            state_box=M.dim_state_constraints_box(ocp),
            control_box=M.dim_control_constraints_box(ocp),
            variable_box=M.dim_variable_constraints_box(ocp),
            path=M.dim_path_constraints_nl(ocp),
            boundary=M.dim_boundary_constraints_nl(ocp),
        ),
    )
end
