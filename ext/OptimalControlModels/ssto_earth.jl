"""
$(TYPEDSIGNATURES)

Constructs an **OptimalControl problem** for the SSTO Earth Launch problem.  
The goal is to minimise the time required to reach a circular orbit at an altitude of 185 km.  

# Arguments

- `::OptimalControlBackend`: Placeholder type specifying the OptimalControl backend or solver interface.
- `grid_size::Int=100`: (Keyword) Number of discretisation points for the direct transcription grid.

# Returns

- `docp`: The direct optimal control problem object representing the SSTO Earth problem.
- `nlp`: The corresponding nonlinear programming model obtained from the DOCP, suitable for numerical optimisation.

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

    ## Scalings
    scaling_p = 1e5
    scaling_v = 1e3
    scaling_m = 1e5

    # model
    ocp = @def begin
        tf ∈ R, variable
        t ∈ [t0, tf], time
        x ∈ R⁵, state
<<<<<<< HEAD
        θ ∈ R, control
=======
        u ∈ R, control
>>>>>>> b16df7e38a3ff4923fb965ed4235c3a8d4521775

        # tf bounds
        tf_l ≤ tf ≤ tf_u
        # control bounds
<<<<<<< HEAD
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
=======
        theta_l ≤ u(t) ≤ theta_u

        # initial conditions
        x(t0) == [0.0, 0.0, 0.0, 0.0, m0]

        # final conditions
        x[2](tf) == y_tf
        x[3](tf) == vx_tf
        x[4](tf) == vy_tf

        # dynamics
        # x[1]=px, x[2]=py, x[3]=vx, x[4]=vy, x[5]=m
        ẋ(t) == [
            x[3](t),
            x[4](t),
            (Thrust * cos(u(t)) - (0.5 * rho_ref * exp(-x[2](t) / h_scale) * sqrt(x[3](t)^2 + x[4](t)^2) * Cd * S) * x[3](t)) / x[5](t),
            (Thrust * sin(u(t)) - (0.5 * rho_ref * exp(-x[2](t) / h_scale) * sqrt(x[3](t)^2 + x[4](t)^2) * Cd * S) * x[4](t)) / x[5](t) - g,
            -Thrust / (g * Isp)
>>>>>>> b16df7e38a3ff4923fb965ed4235c3a8d4521775
        ]

        tf → min
    end

    # initial guess: linear interpolation
    tf_init = 150.0
    px_tf_guess = 5.0e5
    m_tf_guess = m0 - (Thrust / (g * Isp)) * 150.0

    x_init = t -> [
        (0.0 + t / tf_init * px_tf_guess) / scaling_p,
        (0.0 + t / tf_init * y_tf) / scaling_p,
        (0.0 + t / tf_init * vx_tf) / scaling_v,
        (0.0 + t / tf_init * vy_tf) / scaling_v,
        (m0 + t / tf_init * (m_tf_guess - m0)) / scaling_m
    ]
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
