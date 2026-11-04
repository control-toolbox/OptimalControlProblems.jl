"""The **Electric Vehicle (EV) trajectory problem** is a benchmark in optimal control for energy-efficient driving.

Builds the `electric_vehicle` problem with the OptimalControlScalarized backend.

```julia-repl
julia> using OptimalControlProblems
julia> electric_vehicle(OptimalControlScalarizedBackend(); grid_size=100)
```

Default parameters and the modeling status are available through `metadata(:electric_vehicle)` and
`status(:electric_vehicle)`.
"""
function OptimalControlProblems.electric_vehicle(
    ::OptimalControlScalarizedBackend,
    description::Symbol...;
    grid_size::Int=grid_size_data(:electric_vehicle),
    parameters::Union{Nothing,NamedTuple}=nothing,
    kwargs...,
)
    OptimalControlProblems._warn_if_draft(:electric_vehicle, :OptimalControlScalarized)

    # parameters
    params = parameters_data(:electric_vehicle, parameters)
    t0 = params[:t0]
    tf = params[:tf]
    b1 = params[:b1]
    b2 = params[:b2]
    h0 = params[:h0]
    h1 = params[:h1]
    h2 = params[:h2]
    α0 = params[:α0]
    α1 = params[:α1]
    α2 = params[:α2]
    α3 = params[:α3]
    x_t0 = params[:x_t0]
    v_t0 = params[:v_t0]
    x_tf = params[:x_tf]
    v_tf = params[:v_tf]

    # model
    ocp = @def begin
        t ∈ [t0, tf], time
        y = (x, v) ∈ R², state
        u ∈ R, control

        x(t0) == x_t0, (x_t0)
        v(t0) == v_t0, (v_t0)
        x(tf) == x_tf, (x_tf)
        v(tf) == v_tf, (v_tf)

        road = α0 + α1 * x(t) + α2 * x(t)^2 + α3 * x(t)^3
        ∂(x)(t) == v(t)
        ∂(v)(t) == h1 * u(t) - h2 * v(t)^2 - h0 - road

        ∫(b1 * u(t) * v(t) + b2 * u(t)^2) → min
    end

    # initial guess
    yinit = [0.1, 0.1]  # [x, v]
    uinit = [0.1]       # [u]
    init = (state=yinit, control=uinit)

    # discretise the optimal control problem
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
