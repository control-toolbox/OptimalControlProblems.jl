"""
$(TYPEDSIGNATURES)

Constructs an **OptimalControl problem** for the SSTO Earth Launch problem.  
The goal is to minimise the time required to reach a circular orbit at an altitude of 185 km.

# Arguments

- `::OptimalControlBackend`: Placeholder type specifying the OptimalControl backend or solver interface.
- `grid_size::Int=100`: (Keyword) Number of discretisation points for the direct transcription grid.

# Returns

- `docp`: The direct optimal control problem object representing the SSTO Earth problem.

# Example

```julia-repl
julia> using OptimalControlProblems

julia> docp = OptimalControlProblems.ssto_earth(OptimalControlBackend(); N=100);
```
"""
function OptimalControlProblems.ssto_earth(
    ::OptimalControlBackend,
    description::Symbol...;
    grid_size::Int=grid_size_data(:ssto_earth),
    parameters::Union{Nothing,NamedTuple}=nothing,
    kwargs...,
)

    # parameters
    params = parameters_data(:ssto_earth, parameters)
    t0 = params[:t0]
    g = params[:g]
    rho_ref = params[:rho_ref]
    h_scale = params[:h_scale]
    Cd = params[:Cd]
    S = params[:S]
    Thrust = params[:Thrust]
    Isp = params[:Isp]
    m0 = params[:m0]
    y_tf = params[:y_tf]
    vx_tf = params[:vx_tf]
    vy_tf = params[:vy_tf]
    tf_l = params[:tf_l]
    tf_u = params[:tf_u]
    theta_l = params[:theta_l]
    theta_u = params[:theta_u]

    # model
    ocp = @def begin
        tf ∈ R, variable
        t ∈ [t0, tf], time
        x = (px, py, vx, vy, m) ∈ R⁵, state
        theta ∈ R, control

        # tf bounds
        tf_l ≤ tf ≤ tf_u
        # control bounds
        theta_l ≤ theta(t) ≤ theta_u

        # initial conditions
        px(t0) == 0.0
        py(t0) == 0.0
        vx(t0) == 0.0
        vy(t0) == 0.0
        m(t0)  == m0

        # final conditions
        py(tf) == y_tf
        vx(tf) == vx_tf
        vy(tf) == vy_tf

        # dynamics
        ẋ(t) == ssto_dynamics(px(t), py(t), vx(t), vy(t), m(t), theta(t), params)

        tf / 100.0 → min
    end

    function ssto_dynamics(px, py, vx, vy, m, theta, params)
        rho_ref = params[:rho_ref]
        h_scale = params[:h_scale]
        Thrust = params[:Thrust]
        Cd = params[:Cd]
        S = params[:S]
        g = params[:g]
        Isp = params[:Isp]

        v_norm = sqrt(vx^2 + vy^2 + 1e-9)
        rho = rho_ref * exp(-py / h_scale)
        
        dpx = vx
        dpy = vy
        dvx = (Thrust * cos(theta) - 0.5 * rho * v_norm * vx * Cd * S) / m
        dvy = (Thrust * sin(theta) - 0.5 * rho * v_norm * vy * Cd * S) / m - g
        dm  = -Thrust / (g * Isp)

        return [dpx, dpy, dvx, dvy, dm]
    end

    # initial guess
    tf_init = 150.0
    init = (
        state = t -> [
            0.0,                                      # px
            (t - t0) / (tf_init - t0) * y_tf,         # py
            (t - t0) / (tf_init - t0) * vx_tf,         # vx
            0.0,                                      # vy
            m0                                        # m
        ],
        control = 0.5,
        variable = tf_init
    )

    # discretise
    docp = direct_transcription(
        ocp,
        description...;
        init=init,
        grid_size=grid_size,
        disc_method=:trapeze,
        kwargs...,
    )

    return docp
end
