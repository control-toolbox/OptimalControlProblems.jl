"""The **Brachistochrone problem** is a classical benchmark in the history of calculus of variations and optimal control.

Builds the `brachistochrone` problem with the OptimalControl backend.

```julia-repl
julia> using OptimalControlProblems
julia> brachistochrone(OptimalControlBackend(); grid_size=100)
```

Default parameters and the modeling status are available through `metadata(:brachistochrone)` and
`OptimalControlProblems.status(:brachistochrone)`.
"""
function OptimalControlProblems.brachistochrone(
    ::OptimalControlBackend,
    description::Symbol...;
    grid_size::Int=grid_size_data(:brachistochrone),
    parameters::Union{Nothing,NamedTuple}=nothing,
    kwargs...,
)
    OptimalControlProblems._warn_if_draft(:brachistochrone, :OptimalControl)
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

    ocp = @def begin
        tf ∈ R, variable
        t ∈ [t0, tf], time

        z = (px, py, v) ∈ R³, state
        u ∈ R, control

        -1.57 ≤ u(t) ≤ 1.57
        0.1 ≤ tf ≤ 20.0

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

    # initial guess: linear interpolation to match JuMP
    tf_guess = 2.0
    init = (
        state=t -> [
            x0 + (t - t0) / (tf_guess - t0) * (xf - x0),
            y0 + (t - t0) / (tf_guess - t0) * (yf - y0),
            v0 + (t - t0) / (tf_guess - t0) * 10.0,
        ],
        control=1.57,
        variable=tf_guess,
    )

    docp = direct_transcription(
        ocp,
        description...;
        lagrange_to_mayer=false,
        init=init,
        grid_size=grid_size,
        disc_method=:trapeze,
        kwargs...,
    )

    return docp
end
