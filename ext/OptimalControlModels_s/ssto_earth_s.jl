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
        x ∈ R⁵, state
        theta ∈ R, control

        # tf bounds
        tf_l ≤ tf ≤ tf_u
        # control bounds
        theta_l ≤ theta(t) ≤ theta_u

        # initial conditions
        x[1](t0) == 0.0
        x[2](t0) == 0.0
        x[3](t0) == 0.0
        x[4](t0) == 0.0
        x[5](t0) == m0

        # final conditions
        x[2](tf) == y_tf
        x[3](tf) == vx_tf
        x[4](tf) == vy_tf

        # dynamics
        # x[1]=px, x[2]=py, x[3]=vx, x[4]=vy, x[5]=m
        ∂(x[1])(t) == x[3](t)
        ∂(x[2])(t) == x[4](t)
        ∂(x[3])(t) == (Thrust * cos(theta(t)) - (0.5 * rho_ref * exp(-x[2](t) / h_scale) * sqrt(x[3](t)^2 + x[4](t)^2) * Cd * S) * x[3](t)) / x[5](t)
        ∂(x[4])(t) == (Thrust * sin(theta(t)) - (0.5 * rho_ref * exp(-x[2](t) / h_scale) * sqrt(x[3](t)^2 + x[4](t)^2) * Cd * S) * x[4](t)) / x[5](t) - g
        ∂(x[5])(t) == -Thrust / (g * Isp)

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
