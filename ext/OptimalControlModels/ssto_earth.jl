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

        # initial conditions
        x(t0) == [0.0, 0.0, 0.0, 0.0, m0]

        # final conditions
        x(tf)[2] == y_tf
        x(tf)[3] == vx_tf
        x(tf)[4] == vy_tf

        # dynamics
        # x[1]=x, x[2]=y, x[3]=vx, x[4]=vy, x[5]=m
        v = sqrt(x[3](t)^2 + x[4](t)^2)
        rho = rho_ref * exp(-x[2](t) / h_scale)
        D = 0.5 * rho * v^2 * Cd * S
        
        # Avoid division by zero for D_cos_gamma and D_sin_gamma
        # gamma is the angle of the velocity vector
        # cos_gamma = vx / v, sin_gamma = vy / v
        # if v is small, we can approximate D_cos_gamma and D_sin_gamma
        
        ẋ(t) == [
            x[3](t),
            x[4](t),
            (Thrust * cos(θ(t)) - (v > 1e-6 ? D * x[3](t) / v : 0.0)) / x[5](t),
            (Thrust * sin(θ(t)) - (v > 1e-6 ? D * x[4](t) / v : 0.0)) / x[5](t) - g,
            -Thrust / (g * Isp)
        ]

        tf → min
    end

    # initial guess
    tf_init = 150.0
    # x(t) = [x, y, vx, vy, m]
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
