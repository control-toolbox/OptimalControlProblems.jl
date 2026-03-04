"""
$(TYPEDSIGNATURES)

Constructs an **OptimalControl problem** for the Multi-Phase Cannonball problem using scalar state components (symbolic version).  
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
function OptimalControlProblems.cannonball_s(
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
        w = (v0, gamma0, rball, tf) ∈ R⁴, variable
        t ∈ [t0, tf], time
        x = (v_mag, gamma, h, r) ∈ R⁴, state
        u ∈ R, control # dummy

        # variables bounds
        v0_l ≤ v0 ≤ v0_u
        gamma0_l ≤ gamma0 ≤ gamma0_u
        r_ball_l ≤ rball ≤ r_ball_u
        tf_l ≤ tf ≤ tf_u

        # KE constraint
        m_eff = (4/3) * π * rho_metal * rball^3
        0.5 * m_eff * v0^2 ≤ KE_max

        # initial conditions
        v_mag(t0) == v0
        gamma(t0) == gamma0
        h(t0) == 0
        r(t0) == 0

        # final conditions
        h(tf) == 0

        # dynamics
        S_eff = π * rball^2
        rho_val = rho0 * exp(-h(t) / hr)
        D_eff = 0.5 * rho_val * v_mag(t)^2 * S_eff * Cd
        
        ∂(v_mag)(t) == -D_eff/m_eff - g * sin(gamma(t))
        ∂(gamma)(t) == -g * cos(gamma(t)) / v_mag(t)
        ∂(h)(t) == v_mag(t) * sin(gamma(t))
        ∂(r)(t) == v_mag(t) * cos(gamma(t))

        r(tf) → max
    end

    # initial guess
    v0_init = 100.0
    gamma0_init = 0.785
    r_ball_init = 0.05
    tf_init = 10.0
    init = (state=[v0_init, gamma0_init, 1.0, 1.0], variable=[v0_init, gamma0_init, r_ball_init, tf_init])

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
