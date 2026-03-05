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
        x ∈ R⁵, state
        θ ∈ R, control

        # tf bounds
        tf_l ≤ tf ≤ tf_u
        # control bounds
        theta_l ≤ θ(t) ≤ theta_u

        # unscaled helpers
        px = x[1](t) * scaling_p
        py = x[2](t) * scaling_p
        vx = x[3](t) * scaling_v
        vy = x[4](t) * scaling_v
        m  = x[5](t) * scaling_m

        # initial conditions (scaled)
        x(t0) == [0.0, 0.0, 0.0, 0.0, m0 / scaling_m]

        # final conditions (scaled)
        x(tf)[2] == y_tf / scaling_p
        x(tf)[3] == vx_tf / scaling_v
        x(tf)[4] == vy_tf / scaling_v

        # dynamics (scaled)
        v_norm = sqrt(vx^2 + vy^2 + 1e-9)
        rho = rho_ref * exp(-py / h_scale)
        
        ẋ(t) == [
            vx / scaling_p,
            vy / scaling_p,
            ((Thrust * cos(θ(t)) - 0.5 * rho * v_norm * vx * Cd * S) / m) / scaling_v,
            ((Thrust * sin(θ(t)) - 0.5 * rho * v_norm * vy * Cd * S) / m - g) / scaling_v,
            (-Thrust / (g * Isp)) / scaling_m
        ]

        tf → min
    end

    # initial guess
    tf_init = 150.0
    x_init = [1e5, 1e5, 4000.0, 1000.0, 100000.0]
    init = (state=x_init, control=[0.5], variable=[tf_init])

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
