"""The Bryson-Denham problem is a constrained minimum-energy transfer problem.

Builds the `bryson_denham` problem with the OptimalControlScalarized backend.

```julia-repl
julia> using OptimalControlProblems
julia> bryson_denham(OptimalControlScalarizedBackend())
```

Default parameters and the modeling status are available through
`problem_metadata(:bryson_denham)` and `OptimalControlProblems.status(:bryson_denham)`.
"""

function OptimalControlProblems.bryson_denham(
    ::OptimalControlScalarizedBackend;
    parameters::Union{Nothing,NamedTuple}=nothing,
    init=nothing,
)
    OptimalControlProblems.Catalogue._warn_if_draft(
        :bryson_denham, :OptimalControlScalarized
    )
    params = parameters_data(:bryson_denham, parameters)
    t0, tf = params[:t0], params[:tf]
    x1_t0, x2_t0 = params[:x1_t0], params[:x2_t0]
    x1_tf, x2_tf = params[:x1_tf], params[:x2_tf]
    x1_max = params[:x1_max]

    ocp = @def begin
        t ∈ [t0, tf], time
        x ∈ R², state  # x1 (position) et x2(speed)
        u ∈ R, control

        x(t0) == [x1_t0, x2_t0]
        x(tf) == [x1_tf, x2_tf]

        x₁(t) ≤ x1_max

        ∂(x₁)(t) == x₂(t)
        ∂(x₂)(t) == u(t)

        ∫(0.5 * u(t)^2) → min
    end

    # initial guess: from the metadata (#377), unless given
    init = OptimalControlProblems.Metadata._initial_guess(:bryson_denham, params, init)

    return OptimalControlProblems.Problems._problem(
        :bryson_denham, :OptimalControlScalarized, ocp, init, params
    )
end
