function _warn_if_draft(problem::Symbol, modeling::Symbol)
    data = Metadata.METADATA_STORAGE[problem]
    statuses = data[:status]
    if get(statuses, modeling, nothing) === :draft
        notes = get(data, :notes, nothing)
        suffix = isnothing(notes) ? "" : ": $(notes)"
        @warn "$(problem) is a draft for $(modeling)$(suffix)"
    end
end

function _validate_status(status::Symbol)
    status in (:valid, :draft, :all) || throw(
        CTBase.Exceptions.IncorrectArgument(
            "status must be :valid, :draft, or :all; got $status"
        ),
    )
    return status
end

function _select_problems(modelings, status::Symbol; require_all::Bool=true)
    _validate_status(status)
    selected = Symbol[]
    for problem in Metadata.PROBLEM_NAMES
        statuses = Metadata.METADATA_STORAGE[problem][:status]
        predicate =
            spec -> begin
                Modelings._problem_exists(problem, spec) &&
                    (status === :all || get(statuses, spec.name, nothing) === status)
            end
        matches = (require_all ? all : any)(predicate, modelings)
        if matches
            push!(selected, problem)
        end
    end
    return sort!(selected)
end

# ------- Available Problems Function -------
"""
$(TYPEDSIGNATURES)

Returns the sorted list of optimal control problems.

Availability comes from the model files present in `ext/<modeling>Models/`; the status
(`:valid` or `:draft`) of each problem in each modeling comes from its metadata.

- `problems()`: problems having the requested `status` in **at least one** modeling.
- `problems(backend)`: problems available in that modeling with the requested `status`.
- `problems(backend1, backend2, ...)`: problems available in **all** the given modelings
  with the requested `status`, which is the list to use to compare modelings.

# Keyword arguments

- `status::Symbol=:valid`: `:valid`, `:draft` or `:all`.

# Returns

- `Vector{Symbol}`: A vector of problem names as symbols.

# Example

```julia-repl
julia> OptimalControlProblems.problems(JuMPBackend(), OptimalControlBackend())
[:balanced_field, :beam, ...]
```
"""
function problems(; status::Symbol=:valid)::Vector{Symbol}
    return _select_problems(Modelings.MODELING_SPECS, status; require_all=false)
end

function problems(
    backend::Modelings.AbstractModelBackend; status::Symbol=:valid
)::Vector{Symbol}
    return _select_problems((Modelings._modeling_spec(backend),), status)
end

function problems(
    first::Modelings.AbstractModelBackend,
    rest::Modelings.AbstractModelBackend...;
    status::Symbol=:valid,
)::Vector{Symbol}
    modelings = (
        Modelings._modeling_spec(first),
        (Modelings._modeling_spec(backend) for backend in rest)...,
    )
    return _select_problems(modelings, status)
end

"""
$(TYPEDSIGNATURES)

Return the status of `problem` in each modeling where it exists.
"""
function status(problem::Symbol)
    haskey(Metadata.METADATA_STORAGE, problem) || throw(
        CTBase.Exceptions.IncorrectArgument(
            "There is no problem named $problem in metadata"
        ),
    )
    return Metadata.METADATA_STORAGE[problem][:status]
end
