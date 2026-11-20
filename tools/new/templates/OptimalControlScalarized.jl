"""TODO: the first sentence of the description.

Builds the `{{problem}}` problem with the OptimalControlScalarized backend.

```julia-repl
julia> using OptimalControlProblems
julia> {{problem}}(OptimalControlScalarizedBackend())
```

Default parameters and the modeling status are available through
`problem_metadata(:{{problem}})` and `OptimalControlProblems.status(:{{problem}})`.
"""
function OptimalControlProblems.{{problem}}(
    ::OptimalControlScalarizedBackend;
    parameters::Union{Nothing,NamedTuple}=nothing,
    init=nothing,
)
    OptimalControlProblems.Catalogue._warn_if_draft(:{{problem}}, :OptimalControlScalarized)

    # parameters
    params = parameters_data(:{{problem}}, parameters)
    t0 = params[:t0]
    tf = params[:tf]
    x_t0 = params[:x_t0]
    x_u = params[:x_u]

    # model, with scalar components only. TODO: the model of the problem, in place of this
    # example
    ocp = @def begin
        t ∈ [t0, tf], time
        x ∈ R, state
        u ∈ R, control

        x(t0) == x_t0
        x(t) ≤ x_u

        ẋ(t) == u(t)

        ∫(u(t)^2) → min
    end

    # initial guess: from the metadata (#377), unless given
    init = OptimalControlProblems.Metadata._initial_guess(:{{problem}}, params, init)

    return OptimalControlProblems.Problems._problem(
        :{{problem}}, :OptimalControlScalarized, ocp, init, params
    )
end
