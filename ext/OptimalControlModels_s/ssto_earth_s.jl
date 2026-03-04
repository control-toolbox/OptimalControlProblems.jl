"""
$(TYPEDSIGNATURES)

Constructs an **OptimalControl problem** for the SSTO Earth Launch problem using scalar state components (symbolic version).  
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
        x = (px, py, vx, vy, m) ∈ R⁵, state
        u = theta ∈ R, control

        # tf bounds
        tf_l ≤ tf ≤ tf_u
        # control bounds
        theta_l ≤ theta(t) ≤ theta_u

        # initial conditions
        px(t0) == 0.0
        py(t0) == 0.0
        vx(t0) == 0.0
        vy(t0) == 0.0
        m(t0) == m0

        # final conditions
        py(tf) == y_tf
        vx(tf) == vx_tf
        vy(tf) == vy_tf

        # dynamics
        v_mag = sqrt(vx(t)^2 + vy(t)^2)
        rho_val = rho_ref * exp(-py(t) / h_scale)
        D_factor = 0.5 * rho_val * v_mag * Cd * S
        
        ∂(px)(t) == vx(t)
        ∂(py)(t) == vy(t)
        ∂(vx)(t) == (Thrust * cos(theta(t)) - D_factor * vx(t)) / m(t)
        ∂(vy)(t) == (Thrust * sin(theta(t)) - D_factor * vy(t)) / m(t) - g
        ∂(m)(t) == -Thrust / (g * Isp)

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
