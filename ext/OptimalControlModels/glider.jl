"""The **hang glider problem** is a classical benchmark in optimal control.

Builds the `glider` problem with the OptimalControl backend.

```julia-repl
julia> using OptimalControlProblems
julia> glider(OptimalControlBackend())
```

Default parameters and the modeling status are available through
`problem_metadata(:glider)` and `OptimalControlProblems.status(:glider)`.
"""
function OptimalControlProblems.glider(
    ::OptimalControlBackend;
    parameters::Union{Nothing,NamedTuple}=nothing,
    init=nothing,
)
    OptimalControlProblems.Catalogue._warn_if_draft(:glider, :OptimalControl)

    # parameters
    params = parameters_data(:glider, parameters)
    t0 = params[:t0]
    x_t0 = params[:x_t0]
    y_t0 = params[:y_t0]
    y_tf = params[:y_tf]
    vx_t0 = params[:vx_t0]
    vx_tf = params[:vx_tf]
    vy_t0 = params[:vy_t0]
    vy_tf = params[:vy_tf]
    u_c = params[:u_c]
    r_t0 = params[:r_t0]
    m = params[:m]
    g = params[:g]
    c0 = params[:c0]
    c1 = params[:c1]
    S = params[:S]
    ρ = params[:ρ]
    cL_min = params[:cL_min]
    cL_max = params[:cL_max]
    tf_l = params[:tf_l]
    x_l = params[:x_l]
    vx_l = params[:vx_l]

    # model
    ocp = @def begin
        tf ∈ R, variable
        t ∈ [t0, tf], time
        z = (x, y, vx, vy) ∈ R⁴, state
        cL ∈ R, control

        # state constraints
        x(t) ≥ x_l, (x_c)
        vx(t) ≥ vx_l, (vx_c)

        # control constraints
        cL_min ≤ cL(t) ≤ cL_max, (cL_c)

        # initial conditions
        x(t0) == x_t0, (x0_t0)
        y(t0) == y_t0, (y0_t0)
        vx(t0) == vx_t0, (vx0_t0)
        vy(t0) == vy_t0, (vy0_t0)

        # final conditions
        tf ≥ tf_l
        y(tf) == y_tf, (yf_tf)
        vx(tf) == vx_tf, (vxf_tf)
        vy(tf) == vy_tf, (vyf_tf)

        # dynamics
        ż(t) == dynamics(x(t), vx(t), vy(t), cL(t))

        # objective
        -x(tf) → min
    end

    function dynamics(x, vx, vy, cL)
        r = (x / r_t0 - 2.5)^2
        UpD = u_c * (1 - r) * exp(-r)
        w = vy - UpD
        v = √(vx^2 + w^2)
        D = 0.5 * (c0 + c1 * (cL^2)) * ρ * S * (v^2)
        L = 0.5 * cL * ρ * S * (v^2)

        ∂x = vx
        ∂y = vy
        ∂vx = -(L * w + D * vx) / (m * v)
        ∂vy = (L * vx - D * w) / (m * v) - g

        return [∂x, ∂y, ∂vx, ∂vy]
    end

    # initial guess: from the metadata (#377), unless given
    init = OptimalControlProblems.Metadata._initial_guess(:glider, params, init)

    return OptimalControlProblems.Problems._problem(:glider, :OptimalControl, ocp, init, params)
end
