"""The **dielectrophoretic particle problem** is a classical time-optimal control benchmark for microfluidic particle manipulation.

Builds the `dielectrophoretic_particle` problem with the OptimalControlScalarized backend.

```julia-repl
julia> using OptimalControlProblems
julia> dielectrophoretic_particle(OptimalControlScalarizedBackend())
```

Default parameters and the modeling status are available through
`problem_metadata(:dielectrophoretic_particle)` and `OptimalControlProblems.status(:dielectrophoretic_particle)`.
"""
function OptimalControlProblems.dielectrophoretic_particle(
    ::OptimalControlScalarizedBackend;
    parameters::Union{Nothing,NamedTuple}=nothing,
    init=nothing,
)
    OptimalControlProblems.Catalogue._warn_if_draft(
        :dielectrophoretic_particle, :OptimalControlScalarized
    )

    # parameters
    params = parameters_data(:dielectrophoretic_particle, parameters)
    t0 = params[:t0]
    x_t0 = params[:x_t0]
    y_t0 = params[:y_t0]
    x_tf = params[:x_tf]
    α = params[:α]
    c = params[:c]
    u_l = params[:u_l]
    u_u = params[:u_u]
    tf_l = params[:tf_l]

    ocp = @def begin
        tf ∈ R, variable
        t ∈ [t0, tf], time
        q = (x, y) ∈ R², state
        u ∈ R, control

        x(t0) == x_t0, (x_t0)
        y(t0) == y_t0, (y_t0)
        x(tf) == x_tf, (x_tf)
        tf ≥ tf_l, (tf_c)
        u_l ≤ u(t) ≤ u_u, (u_c)

        ∂(x)(t) == y(t) * u(t) + α * u(t)^2
        ∂(y)(t) == -c * y(t) + u(t)

        tf → min
    end

    # initial guess: from the metadata (#377), unless given
    init = OptimalControlProblems.Metadata._initial_guess(:dielectrophoretic_particle, params, init)

    return OptimalControlProblems.Problems._problem(:dielectrophoretic_particle, :OptimalControlScalarized, ocp, init, params)
end
