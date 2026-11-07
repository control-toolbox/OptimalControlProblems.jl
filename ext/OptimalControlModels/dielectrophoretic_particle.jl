"""The **dielectrophoretic particle problem** is a classical time-optimal control benchmark for microfluidic particle manipulation.

Builds the `dielectrophoretic_particle` problem with the OptimalControl backend.

```julia-repl
julia> using OptimalControlProblems
julia> dielectrophoretic_particle(OptimalControlBackend())
```

Default parameters and the modeling status are available through
`problem_metadata(:dielectrophoretic_particle)` and `OptimalControlProblems.status(:dielectrophoretic_particle)`.
"""
function OptimalControlProblems.dielectrophoretic_particle(
    ::OptimalControlBackend; parameters::Union{Nothing,NamedTuple}=nothing
)
    OptimalControlProblems._warn_if_draft(:dielectrophoretic_particle, :OptimalControl)

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

        q̇(t) == dynamics(y(t), u(t))

        tf → min
    end

    function dynamics(y, u)
        return [y * u + α * u^2, -c * y + u]
    end

    # initial guess
    init = (state=[1, 0.5], control=0.5, variable=8)

    return OptimalControlProblems._problem(
        :dielectrophoretic_particle, :OptimalControl, ocp, init, params
    )
end
