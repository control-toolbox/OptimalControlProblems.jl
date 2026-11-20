"""The **Brachistochrone problem** is a classical benchmark in the history of calculus of variations and optimal control.

Builds the `brachistochrone` problem with the OptimalControl backend.

```julia-repl
julia> using OptimalControlProblems
julia> brachistochrone(OptimalControlBackend())
```

Default parameters and the modeling status are available through
`problem_metadata(:brachistochrone)` and `OptimalControlProblems.status(:brachistochrone)`.
"""
function OptimalControlProblems.brachistochrone(
    ::OptimalControlBackend; parameters::Union{Nothing,NamedTuple}=nothing, init=nothing
)
    OptimalControlProblems.Catalogue._warn_if_draft(:brachistochrone, :OptimalControl)
    params = parameters_data(:brachistochrone, parameters)
    g = params[:g]
    t0 = params[:t0]
    x0 = params[:x0]
    y0 = params[:y0]
    v0 = params[:v0]
    xf = params[:xf]
    yf = params[:yf]
    u_min = params[:u_min]
    u_max = params[:u_max]
    tf_l = params[:tf_l]
    tf_u = params[:tf_u]

    ocp = @def begin
        tf ∈ R, variable
        t ∈ [t0, tf], time

        z = (px, py, v) ∈ R³, state
        u ∈ R, control

        u_min ≤ u(t) ≤ u_max
        tf_l ≤ tf ≤ tf_u

        px(t0) == x0
        py(t0) == y0
        v(t0) == v0

        px(tf) == xf
        py(tf) == yf

        ∂(px)(t) == v(t) * sin(u(t))
        ∂(py)(t) == -v(t) * cos(u(t))
        ∂(v)(t) == g * cos(u(t))

        # Objectif
        tf → min
    end

    # initial guess: from the metadata (#377), unless given
    init = OptimalControlProblems.Metadata._initial_guess(:brachistochrone, params, init)

    return OptimalControlProblems.Problems._problem(
        :brachistochrone, :OptimalControl, ocp, init, params
    )
end
