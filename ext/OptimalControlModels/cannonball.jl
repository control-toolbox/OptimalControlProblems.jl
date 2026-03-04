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
    ρ_metal = params[:ρ_metal]
    Cd = params[:Cd]
    KE_max = params[:KE_max]
    g = params[:g]
    ρ0 = params[:ρ0]
    hr = params[:hr]
    r_ball_l = params[:r_ball_l]
    r_ball_u = params[:r_ball_u]
    v0_l = params[:v0_l]
    v0_u = params[:v0_u]
    γ0_l = params[:γ0_l]
    γ0_u = params[:γ0_u]
    tf_l = params[:tf_l]
    tf_u = params[:tf_u]

    # model
    ocp = @def begin
        vars ∈ R³, variable
        v0 = vars[1]
        γ0 = vars[2]
        r_ball = vars[3]
        tf ∈ R, variable
        t ∈ [t0, tf], time
        x ∈ R⁴, state

        # variables bounds
        v0_l ≤ v0 ≤ v0_u
        γ0_l ≤ γ0 ≤ γ0_u
        r_ball_l ≤ r_ball ≤ r_ball_u
        tf_l ≤ tf ≤ tf_u

        # KE constraint: 0.5 * m * v0^2 ≤ KE_max
        # m = (4/3) * π * ρ_metal * r_ball^3
        0.5 * ((4/3) * π * ρ_metal * r_ball^3) * v0^2 ≤ KE_max

        # initial conditions
        x(t0) == [v0, γ0, 0, 0]

        # final conditions
        x(tf)[3] == 0 # h(tf) = 0

        # dynamics
        m = (4/3) * π * ρ_metal * r_ball^3
        S = π * r_ball^2
        
        # x[1]=v, x[2]=γ, x[3]=h, x[4]=r
        ẋ(t) == [
            -(0.5 * ρ0 * exp(-x[3](t) / hr) * x[1](t)^2 * S * Cd) / m - g * sin(x[2](t)),
            -g * cos(x[2](t)) / x[1](t),
            x[1](t) * sin(x[2](t)),
            x[1](t) * cos(x[2](t))
        ]

        x(tf)[4] → max
    end

    # initial guess
    v0_init = 100.0
    γ0_init = π/4
    r_ball_init = 0.05
    tf_init = 10.0
    init = (state=[v0_init, γ0_init, 1.0, 1.0], variable=[v0_init, γ0_init, r_ball_init, tf_init])

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
