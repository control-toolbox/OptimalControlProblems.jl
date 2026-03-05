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
        theta_l ≤ θ(t) ≤ theta_u

        # unscaled helpers
        px = spx(t) * scaling_p
        py = spy(t) * scaling_p
        vx = svx(t) * scaling_v
        vy = svy(t) * scaling_v
        m  = sm(t) * scaling_m

        # initial conditions (scaled)
        spx(t0) == 0
        spy(t0) == 0
        svx(t0) == 0
        svy(t0) == 0
        sm(t0) == m0 / scaling_m

        # final conditions (scaled)
        spy(tf) == y_tf / scaling_p
        svx(tf) == vx_tf / scaling_v
        svy(tf) == vy_tf / scaling_v

        # dynamics (scaled)
        v_norm = sqrt(vx^2 + vy^2 + 1e-9)
        rho = rho_ref * exp(-py / h_scale)
        
        ∂(spx)(t) == vx / scaling_p
        ∂(spy)(t) == vy / scaling_p
        ∂(svx)(t) == ((Thrust * cos(θ(t)) - 0.5 * rho * v_norm * vx * Cd * S) / m) / scaling_v
        ∂(svy)(t) == ((Thrust * sin(θ(t)) - 0.5 * rho * v_norm * vy * Cd * S) / m - g) / scaling_v
        ∂(sm)(t) == (-Thrust / (g * Isp)) / scaling_m

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
