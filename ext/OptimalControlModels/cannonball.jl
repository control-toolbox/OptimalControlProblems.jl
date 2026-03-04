"""
$(TYPEDSIGNATURES)

Constructs an **OptimalControl problem** for the Multi-Phase Cannonball problem.  
The goal is to maximise the total range of a cannonball by optimizing its radius, initial velocity, and launch angle, subject to a maximum muzzle energy constraint.  

# Arguments

- `::OptimalControlBackend`: Placeholder type specifying the OptimalControl backend or solver interface.
- `grid_size::Int=100`: (Keyword) Number of discretisation points for the direct transcription grid.

# Returns

- `docp`: The direct optimal control problem object representing the cannonball problem.
- `nlp`: The corresponding nonlinear programming model obtained from the DOCP, suitable for numerical optimisation.

# Example

```julia-repl
julia> using OptimalControlProblems

julia> docp = OptimalControlProblems.cannonball(OptimalControlBackend(); N=100);
```
"""
function OptimalControlProblems.cannonball(
    ::OptimalControlBackend,
    description::Symbol...;
    grid_size::Int=grid_size_data(:cannonball),
    parameters::Union{Nothing,NamedTuple}=nothing,
    kwargs...,
)

    # parameters
    params = parameters_data(:cannonball, parameters)
    t0 = params[:t0]
    rho_metal = params[:rho_metal]
    Cd = params[:Cd]
    KE_max = params[:KE_max]
    g = params[:g]
    rho0 = params[:rho0]
    hr = params[:hr]
    r_ball_l = params[:r_ball_l]
    r_ball_u = params[:r_ball_u]
    v0_l = params[:v0_l]
    v0_u = params[:v0_u]
    gamma0_l = params[:gamma0_l]
    gamma0_u = params[:gamma0_u]
    tf_l = params[:tf_l]
    tf_u = params[:tf_u]

    # model
    ocp = @def begin
        w = (tf, v0, gamma0, rball) ∈ R⁴, variable
        t ∈ [t0, tf], time
        x ∈ R⁴, state
        u ∈ R, control # dummy

        # variables bounds
        tf_l ≤ tf ≤ tf_u
        v0_l ≤ w[2] ≤ v0_u
        gamma0_l ≤ w[3] ≤ gamma0_u
        r_ball_l ≤ w[4] ≤ r_ball_u

        # KE constraint: 0.5 * m * v0^2 ≤ KE_max
        # m = (4/3) * π * rho_metal * rball^3
        0.5 * ((4/3) * π * rho_metal * w[4]^3) * w[2]^2 ≤ KE_max

        # initial conditions
        x[1](t0) == w[2]
        x[2](t0) == w[3]
        x[3](t0) == 0.0
        x[4](t0) == 0.0

        # final conditions
        x[3](tf) == 0.0

        # dynamics
        # x[1]=v_mag, x[2]=gamma, x[3]=h, x[4]=r
        m_eff = (4/3) * π * rho_metal * w[4]^3
        S_eff = π * w[4]^2
        
        ẋ(t) == [
            -(0.5 * rho0 * exp(-x[3](t) / hr) * x[1](t)^2 * S_eff * Cd) / m_eff - g * sin(x[2](t)),
            -g * cos(x[2](t)) / x[1](t),
            x[1](t) * sin(x[2](t)),
            x[1](t) * cos(x[2](t))
        ]

        x[4](tf) → max
    end

    # initial guess
    v0_init = 100.0
    gamma0_init = 0.785
    r_ball_init = 0.05
    tf_init = 10.0
    init = (state=[v0_init, gamma0_init, 1.0, 1.0], variable=[tf_init, v0_init, gamma0_init, r_ball_init])

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
