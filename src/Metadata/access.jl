"""
$(TYPEDSIGNATURES)

Return the dictionary containing the metadata of all available optimal control problems.

# Example

```julia-repl
julia> problem_metadata()
```
"""
problem_metadata() = METADATA_STORAGE

"""
$(TYPEDSIGNATURES)

Return a dictionary containing the metadata of `problem`, without building it.

The metadata of a built problem is read with `metadata(pb)` on the [`Problem`](@ref OptimalControlProblems.Problems.Problem).
The following keys are valid:

- `grid_size::Int`: the default number of steps. For example:
```julia
:grid_size => 500,
```
- `grid_sizes::NamedTuple` (optional): the number of steps of the problem in some modelings,
  where it differs from the default of the modeling (see `grid_size_data`). For example:
```julia
:grid_sizes => (ModelingToolkit=100,),
```
- `parameters::Union{Nothing,NamedTuple}`: the list of parameters. For example:
```julia
:parameters => (
    t0 = 0,
    tf = 1,
    x₁_l = 0,
    x₁_u = 0.1,
    x₁_t0 = 0,
    x₂_t0 = 1,
    x₁_tf = 0,
    x₂_tf = -1,
),
```
- `parameter_info`: the unit and the meaning of each parameter, in the order of
  `parameters`; `unit` is `nothing` for a dimensionless parameter. For example:
```julia
:parameter_info => (
    t0 = (unit = "s", meaning = "initial time"),
    x₁_u = (unit = nothing, meaning = "upper bound of the state x₁"),
    ...
),
```
- `status`: the status (`:valid` or `:draft`) of the problem in each modeling.
- `init`: the initial guess of every modeling, a function of the parameters (see
  `OptimalControlProblems.initial_guess_data`).
- `reference_solution` (optional): the analytic solution of the continuous problem, a
  function of the parameters; see [`OptimalControlProblems.reference_solution_data`](@ref OptimalControlProblems.References.reference_solution_data).

# Example

```julia-repl
julia> data = problem_metadata(:beam)
julia> data[:grid_size]
500
```
"""
function problem_metadata(problem::Symbol)
    haskey(METADATA_STORAGE, problem) || throw(
        CTBase.Exceptions.IncorrectArgument(
            "unknown problem";
            got="$(problem)",
            expected="one of the problems of the collection",
            suggestion="list them with problems(; status=:all)",
        ),
    )
    return METADATA_STORAGE[problem]
end

"""
$(TYPEDSIGNATURES)

Return the number of discretisation steps, from the metadata, for a given optimal control problem.

# Arguments

- `problem::Symbol`: The name of the problem.

# Returns

- `Int`: The number of discretisation steps (`N`) of the specified problem.

# Example

```julia-repl
julia> grid_size_data(:beam)
500
```
"""
function grid_size_data(problem::Symbol)
    return problem_metadata(problem)[:grid_size]
end

"""
The default number of steps of a ModelingToolkit problem: it is solved by direct shooting
(Corleone), one integration per control interval, which needs a coarser grid than a
collocation (#435). At most the grid of the problem.
"""
const MODELING_TOOLKIT_GRID_SIZE = 50

_grid_size(problem::Symbol, ::Modelings.AbstractModelBackend) = grid_size_data(problem)
function _grid_size(problem::Symbol, ::Modelings.ModelingToolkitBackend)
    return min(grid_size_data(problem), MODELING_TOOLKIT_GRID_SIZE)
end

"""
$(TYPEDSIGNATURES)

Return the default number of steps of `problem` in the modeling of `backend`: the entry of
the modeling in the field `:grid_sizes` of the metadata when there is one, otherwise the
default of the modeling, the grid of the problem (`grid_size_data(problem)`) but for
ModelingToolkit, solved by shooting on a coarser grid (`MODELING_TOOLKIT_GRID_SIZE`).

# Example

```julia-repl
julia> grid_size_data(:beam, JuMPBackend())
500
```
"""
function grid_size_data(problem::Symbol, backend::Modelings.AbstractModelBackend)
    grid_sizes = get(problem_metadata(problem), :grid_sizes, (;))
    name = Modelings._modeling_spec(backend).name
    return haskey(grid_sizes, name) ? grid_sizes[name] : _grid_size(problem, backend)
end

"""
$(TYPEDSIGNATURES)

Return the parameter set associated with a given problem.

# Arguments

- `problem::Symbol`: the name of the problem whose parameters are requested.

# Returns

- `::Union{Nothing,NamedTuple}`: the parameters of the problem, or `nothing` if none exist.

# Example

```julia-repl
julia> parameters_data(:beam)
(t0 = 0, tf = 1, ...)
```
"""
function parameters_data(problem::Symbol)
    return problem_metadata(problem)[:parameters]
end

"""
$(TYPEDSIGNATURES)

Return the parameter set associated with a given problem, optionally merged with user-supplied parameters.

# Arguments

- `problem::Symbol`: the name of the problem.
- `parameters::Union{Nothing,NamedTuple}`: user-supplied parameters to override or extend the defaults.  
  If `nothing`, returns the default parameters unchanged.

# Returns

- `::Union{Nothing,NamedTuple}`: the merged parameters.  
  Throws `CTBase.Exceptions.PreconditionError` if parameters are given to a problem that has none.

# Example

```julia-repl
julia> parameters_data(:beam, (tf = 2,))
(t0 = 0, tf = 2, ...)
```
"""
function parameters_data(problem::Symbol, parameters::Union{Nothing,NamedTuple})
    defaults = parameters_data(problem)
    isnothing(parameters) && return defaults
    isnothing(defaults) && throw(
        CTBase.Exceptions.PreconditionError(
            "there are no parameters to merge";
            reason="problem $(problem) has no parameters",
            suggestion="call $(problem) without the parameters keyword",
        ),
    )
    return Base.merge(defaults, parameters)
end

"""
$(TYPEDSIGNATURES)

Return the initial guess of `problem` for its default parameters, merged with `parameters`
if given: `(state=…, control=…, variable=…)`, each component a constant, a vector, or a
function of the time `t`.

The initial guess is shared by every modeling (#377): it is a function of the parameters in
the metadata of the problem (`ext/MetaData/<problem>.jl`). The OptimalControl modelings pass
it as is; JuMP evaluates it at the nodes of the scheme. Every modeling takes another one with
the keyword `init` when the problem is built.

# Example

```julia-repl
julia> OptimalControlProblems.initial_guess_data(:beam)
(state = [0.05, 0.1], control = 0.1)
```
"""
function initial_guess_data(problem::Symbol, parameters::Union{Nothing,NamedTuple}=nothing)
    return problem_metadata(problem)[:init](parameters_data(problem, parameters))
end

# The initial guess a problem is built with: `init` if given, otherwise the one of the
# metadata for the parameters `params` of the problem.
function _initial_guess(problem::Symbol, params, init)
    isnothing(init) || return init
    return problem_metadata(problem)[:init](params)
end
