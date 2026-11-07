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
$(TYPEDSIGNATURES)

Merge two `Nothing` values.

# Arguments

- `::Nothing`: first argument.
- `::Nothing`: second argument.

# Returns

- `nothing::Nothing`: always returns `nothing`.

# Example

```julia-repl
julia> merge(nothing, nothing)
nothing
```
"""
merge(::Nothing, ::Nothing) = nothing

"""
$(TYPEDSIGNATURES)

Merge a `NamedTuple` with `nothing`.  

# Arguments

- `A::NamedTuple`: a named tuple to keep.
- `::Nothing`: placeholder, ignored.

# Returns

- `::NamedTuple`: returns `A` unchanged.

# Example

```julia-repl
julia> merge((a=1,), nothing)
(a = 1,)
```
"""
merge(A::NamedTuple, ::Nothing) = A

"""
$(TYPEDSIGNATURES)

Throw an error when attempting to merge `nothing` with a `NamedTuple`.  

# Arguments

- `::Nothing`: indicates there is no data to merge.
- `::NamedTuple`: the data that cannot be merged.

# Returns

- This function always throws `CTBase.Exceptions.PreconditionError`.

# Example

```julia-repl
julia> merge(nothing, (a=1,))
ERROR: PreconditionError: there is nothing to merge
```
"""
function merge(::Nothing, ::NamedTuple)
    return throw(
        CTBase.Exceptions.PreconditionError(
            "there is nothing to merge";
            reason="parameters are given to update, but there are no defaults to update",
        ),
    )
end

"""
$(TYPEDSIGNATURES)

Merge two `NamedTuple`s, with the second one overriding keys from the first when duplicated.

# Arguments

- `A::NamedTuple`: first set of key–value pairs.
- `B::NamedTuple`: second set of key–value pairs, takes precedence if keys overlap.

# Returns

- `::NamedTuple`: merged named tuple containing keys from both `A` and `B`.

# Example

```julia-repl
julia> merge((a=1, b=2), (b=3, c=4))
(a = 1, b = 3, c = 4)
```
"""
function merge(A::NamedTuple, B::NamedTuple)
    f(; kwargs...) = kwargs
    return NamedTuple(f(; A..., B...))
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
    try
        return merge(parameters_data(problem), parameters)
    catch e
        if e isa CTBase.Exceptions.PreconditionError
            throw(
                CTBase.Exceptions.PreconditionError(
                    "there are no parameters to merge";
                    reason="problem $(problem) has no parameters",
                    suggestion="call $(problem) without the parameters keyword",
                ),
            )
        else
            rethrow(e)
        end
    end
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
