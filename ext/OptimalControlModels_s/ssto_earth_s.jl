"""
$(TYPEDSIGNATURES)

Constructs an **OptimalControl problem** for the SSTO Earth Launch problem using scalar state components (symbolic version).  
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
function OptimalControlProblems.ssto_earth_s(
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
        (px, py, vx, vy, m) ∈ R⁵, state
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
        v_norm = sqrt(vx(t)^2 + vy(t)^2 + 1e-9)
        rho = rho_ref * exp(-py(t) / h_scale)
        
        ∂(px)(t) == vx(t)
        ∂(py)(t) == vy(t)
        ∂(vx)(t) == (Thrust * cos(theta(t)) - 0.5 * rho * v_norm * vx(t) * Cd * S) / m(t)
        ∂(vy)(t) == (Thrust * sin(theta(t)) - 0.5 * rho * v_norm * vy(t) * Cd * S) / m(t) - g
        ∂(m)(t)  == -Thrust / (g * Isp)

        tf / 100.0 → min
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
