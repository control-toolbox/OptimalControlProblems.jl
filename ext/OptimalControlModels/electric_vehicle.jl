"""The **Electric Vehicle (EV) trajectory problem** is a benchmark in optimal control for energy-efficient driving.

Builds the `electric_vehicle` problem with the OptimalControl backend.

```julia-repl
julia> using OptimalControlProblems
julia> electric_vehicle(OptimalControlBackend())
```

Default parameters and the modeling status are available through
`problem_metadata(:electric_vehicle)` and `OptimalControlProblems.status(:electric_vehicle)`.
"""
function OptimalControlProblems.electric_vehicle(
    ::OptimalControlBackend;
    parameters::Union{Nothing,NamedTuple}=nothing,
    init=nothing,
)
    OptimalControlProblems._warn_if_draft(:electric_vehicle, :OptimalControl)

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

        ẏ(t) == dynamics(x(t), v(t), u(t))

        ∫(b1 * u(t) * v(t) + b2 * u(t)^2) → min
    end

    road(x) = α0 + α1 * x + α2 * x^2 + α3 * x^3
    dynamics(x, v, u) = [v, h1 * u - h2 * v^2 - h0 - road(x)]

    # initial guess: from the metadata (#377), unless given
    init = OptimalControlProblems._initial_guess(:electric_vehicle, params, init)

    return OptimalControlProblems._problem(:electric_vehicle, :OptimalControl, ocp, init, params)
end
