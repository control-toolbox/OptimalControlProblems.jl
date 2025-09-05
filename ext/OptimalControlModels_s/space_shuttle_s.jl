"""
$(TYPEDSIGNATURES)

Constructs an **OptimalControl problem** for the Space Shuttle reentry trajectory.  
This function defines the state variables (altitude, longitude, latitude, velocity, flight path angle, azimuth), the control variables (angle of attack, bank angle), system dynamics, constraints, initial and terminal conditions, and the cost functional, which maximises the latitude (cross range) at the terminal point.  
Reference: Original JuMP model formulation [here](https://jump.dev/JuMP.jl/stable/tutorials/nonlinear/space_shuttle_reentry_trajectory/).  
Note: No heating limit path constraint is included.

# Arguments

- `::OptimalControlBackend`: Placeholder type specifying the OptimalControl backend or solver interface.
- `N::Int=500`: (Keyword) Number of discretisation points for the direct transcription grid.

# Returns

- `docp`: The direct optimal control problem object representing the Space Shuttle reentry trajectory.
- `nlp`: The corresponding nonlinear programming model obtained from the DOCP, suitable for numerical optimisation.

# Example

```julia-repl
julia> using OptimalControlProblems

julia> docp = OptimalControlProblems.space_shuttle(OptimalControlBackend(); N=500);
```
"""
function OptimalControlProblems.space_shuttle_s(
    ::OptimalControlBackend,
    description::Symbol...;
    N::Int=steps_number_data(:space_shuttle),
    kwargs...,
)

    ## Global variables
    w = 203000.0  # weight (lb)
    g₀ = 32.174    # acceleration (ft/sec^2)
    m = w / g₀    # mass (slug)

    ## Aerodynamic and atmospheric forces on the vehicle
    ρ₀ = 0.002378
    hᵣ = 23800
    Rₑ = 20902900
    μ = 0.14076539e17
    S = 2690
    a₀ = -0.20704
    a₁ = 0.029244
    b₀ = 0.07854
    b₁ = -0.61592e-2
    b₂ = 0.621408e-3

    # 
    Δt_min = 3.5
    Δt_max = 4.5
    tf_min = 500*Δt_min
    tf_max = 500*Δt_max

    ## Initial conditions
    h_s = 2.6          # altitude (ft) / 1e5
    ϕ_s = deg2rad(0)   # longitude (rad)
    θ_s = deg2rad(0)   # latitude (rad)
    v_s = 2.56         # velocity (ft/sec) / 1e4
    γ_s = deg2rad(-1)  # flight path angle (rad)
    ψ_s = deg2rad(90)  # azimuth (rad)
    α_s = deg2rad(0)   # angle of attack (rad)
    β_s = deg2rad(0)   # bank angle (rad)

    ## Final conditions, the so-called Terminal Area Energy Management (TAEM)
    h_t = 0.8          # altitude (ft) / 1e5
    v_t = 0.25         # velocity (ft/sec) / 1e4
    γ_t = deg2rad(-5)  # flight path angle (rad)

    ## Scalings
    scaling_h = 1e5
    scaling_v = 1e4

    # model
    ocp = @def begin

        ## define the problem
        tf ∈ R, variable
        t ∈ [0, tf], time
        x = (scaled_h, ϕ, θ, scaled_v, γ, ψ) ∈ R⁶, state
        u = (α, β) ∈ R², control

        ## constraints
        # to help convergence and avoid domain value error
        -2π ≤ ϕ(t) ≤ 2π
        -2π ≤ ψ(t) ≤ 2π

        # final time constraints
        tf_min ≤ tf ≤ tf_max

        # state constraints
        0 ≤ scaled_h(t) ≤ Inf, (scaled_h_con)
        deg2rad(-89) ≤ θ(t) ≤ deg2rad(89), (θ_con)
        0 ≤ scaled_v(t) ≤ Inf, (scaled_v_con)
        deg2rad(-89) ≤ γ(t) ≤ deg2rad(89), (γ_con)

        # control constraints
        deg2rad(-90) ≤ α(t) ≤ deg2rad(90), (α_con)
        deg2rad(-89) ≤ β(t) ≤ deg2rad(1), (β_con)

        # initial conditions
        scaled_h(0) == h_s, (scaled_h0_con)
        ϕ(0) == ϕ_s, (ϕ0_con)
        θ(0) == θ_s, (θ0_con)
        scaled_v(0) == v_s, (scaled_v0_con)
        γ(0) == γ_s, (γ0_con)
        ψ(0) == ψ_s, (ψ0_con)

        # final conditions
        scaled_h(tf) == h_t, (scaled_hf_con)
        scaled_v(tf) == v_t, (scaled_vf_con)
        γ(tf) == γ_t, (γf_con)

        ## Helper functions
        h = scaled_h(t) * scaling_h
        v = scaled_v(t) * scaling_v
        r2dα = α(t) * (180 / π)
        c_D = b₀ + b₁ * r2dα + b₂ * r2dα^2
        c_L = a₀ + a₁ * r2dα
        ρ = ρ₀ * exp(-h / hᵣ)
        D = (1 / 2) * c_D * S * ρ * (v^2)
        L = (1 / 2) * c_L * S * ρ * (v^2)
        r = Rₑ + h
        g = μ / (r^2)

        ## dynamics
        ∂(scaled_h)(t) == v * sin(γ(t)) / scaling_h
        ∂(ϕ)(t) == (v / r) * cos(γ(t)) * sin(ψ(t)) / cos(θ(t))
        ∂(θ)(t) == (v / r) * cos(γ(t)) * cos(ψ(t))
        ∂(scaled_v)(t) == (-(D / m) - g * sin(γ(t))) / scaling_v
        ∂(γ)(t) == (L / (m * v)) * cos(β(t)) + cos(γ(t)) * ((v / r) - (g / v))
        ∂(ψ)(t) == (1 / (m * v * cos(γ(t)))) * L * sin(β(t)) + (v / (r * cos(θ(t)))) * cos(γ(t)) * sin(ψ(t)) * sin(θ(t))

        ## objective
        -θ(tf) → min
    end


    # initial guess: linear interpolation for h, v, gamma (NB. t0 = 0), constant for the rest
    # variable time step seems to be initialized at 1 in jump
    # note that ipopt will project the initial guess inside the bounds anyway.
    tf_init = (tf_min+tf_max)/2
    x_init =
        t -> [
            h_s + t / tf_init * (h_t - h_s),
            ϕ_s,
            θ_s,
            v_s + t / tf_init * (v_t - v_s),
            γ_s + t / tf_init * (γ_t - γ_s),
            ψ_s,
        ]
    init = (state=x_init, control=[α_s, β_s], variable=[tf_init])

    # discretise the optimal control problem
    docp = direct_transcription(
        ocp,
        description...;
        lagrange_to_mayer=false,
        init=init,
        grid_size=N,
        disc_method=:trapeze,
        kwargs...,
    )

    return docp
end
