"""
$(TYPEDSIGNATURES)

Constructs an **OptimalControl problem** for the Space Shuttle reentry trajectory.  
This function defines the state variables (altitude, longitude, latitude, velocity, flight path angle, azimuth), the control variables (angle of attack, bank angle), system dynamics, constraints, initial and terminal conditions, and the cost functional, which maximises the latitude (cross range) at the terminal point.  
Reference: Original JuMP model formulation [here](https://jump.dev/JuMP.jl/stable/tutorials/nonlinear/space_shuttle_reentry_trajectory/).  
Note: No heating limit path constraint is included.

# Arguments

- `::OptimalControlBackend`: Placeholder type specifying the OptimalControl backend or solver interface.
- `grid_size::Int=500`: (Keyword) Number of discretisation points for the direct transcription grid.

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
    grid_size::Int=grid_size_data(:space_shuttle),
    parameters::Union{Nothing,NamedTuple}=nothing,
    kwargs...,
)

    # Parameters
    params = parameters_data(:space_shuttle, parameters)
    t0 = params[:t0]

    ##
    w = params[:w]
    g₀ = params[:g₀]
    m = w / g₀      # mass (slug)

    ## Aerodynamic and atmospheric forces on the vehicle
    ρ₀ = params[:ρ₀]
    hᵣ = params[:hᵣ]
    Rₑ = params[:Rₑ]
    μ = params[:μ]
    S = params[:S]
    a₀ = params[:a₀]
    a₁ = params[:a₁]
    b₀ = params[:b₀]
    b₁ = params[:b₁]
    b₂ = params[:b₂]

    # 
    Δt_min = params[:Δt_min]
    Δt_max = params[:Δt_max]
    tf_l = 1500.0  #
    tf_u = 2500.0  #

    ## Initial conditions
    h_t0 = params[:h_t0]
    ϕ_t0 = params[:ϕ_t0]
    θ_t0 = params[:θ_t0]
    v_t0 = params[:v_t0]
    γ_t0 = params[:γ_t0]
    ψ_t0 = params[:ψ_t0]

    # for initial guess
    α_s = params[:α_s]
    β_s = params[:β_s]

    ## Final conditions, the so-called Terminal Area Energy Management (TAEM)
    h_tf = params[:h_tf]
    v_tf = params[:v_tf]
    γ_tf = params[:γ_tf]

    ##
    h_l = params[:h_l]
    ϕ_l = params[:ϕ_l]
    ϕ_u = params[:ϕ_u]
    θ_l = params[:θ_l]
    θ_u = params[:θ_u]
    v_l = params[:v_l]
    γ_l = params[:γ_l]
    γ_u = params[:γ_u]
    ψ_l = params[:ψ_l]
    ψ_u = params[:ψ_u]
    α_l = params[:α_l]
    α_u = params[:α_u]
    β_l = params[:β_l]
    β_u = params[:β_u]

    ## Scalings
    scaling_h = 1e5
    scaling_v = 1e4

    # model
    ocp = @def begin
        # define the problem
        tf ∈ R, variable
        t ∈ [t0, tf], time
        x = (scaled_h, ϕ, θ, scaled_v, γ, ψ) ∈ R⁶, state
        u = (α, β) ∈ R², control

        # constraints
        ## to help convergence and avoid domain value error
        ϕ_l ≤ ϕ(t) ≤ ϕ_u
        ψ_l ≤ ψ(t) ≤ ψ_u

        ## final time constraints
        tf_l ≤ tf ≤ tf_u

        ## state constraints
        scaled_h(t) ≥ h_l, (scaled_h_c)
        θ_l ≤ θ(t) ≤ θ_u, (θ_c)
        scaled_v(t) ≥ v_l, (scaled_v_c)
        γ_l ≤ γ(t) ≤ γ_u, (γ_c)

        ## control constraints
        α_l ≤ α(t) ≤ α_u, (α_c)
        β_l ≤ β(t) ≤ β_u, (β_c)

        ## initial conditions
        scaled_h(t0) == h_t0, (scaled_h_t0)
        ϕ(t0) == ϕ_t0, (ϕ_t0)
        θ(t0) == θ_t0, (θ_t0)
        scaled_v(t0) == v_t0, (scaled_v_t0)
        γ(t0) == γ_t0, (γ_t0)
        ψ(t0) == ψ_t0, (ψ_t0)

        ## final conditions
        scaled_h(tf) == h_tf, (scaled_h_tf)
        scaled_v(tf) == v_tf, (scaled_v_tf)
        γ(tf) == γ_tf, (γ_tf)

        # Helper functions
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

        # dynamics
        ∂(scaled_h)(t) == v * sin(γ(t)) / scaling_h
        ∂(ϕ)(t) == (v / r) * cos(γ(t)) * sin(ψ(t)) / cos(θ(t))
        ∂(θ)(t) == (v / r) * cos(γ(t)) * cos(ψ(t))
        ∂(scaled_v)(t) == (-(D / m) - g * sin(γ(t))) / scaling_v
        ∂(γ)(t) == (L / (m * v)) * cos(β(t)) + cos(γ(t)) * ((v / r) - (g / v))
        ∂(ψ)(t) ==
        (1 / (m * v * cos(γ(t)))) * L * sin(β(t)) +
        (v / (r * cos(θ(t)))) * cos(γ(t)) * sin(ψ(t)) * sin(θ(t))

        # objective
        -θ(tf) → min
    end

    # initial guess: linear interpolation for h, v, gamma (NB. t0 = 0), constant for the rest
    # variable time step seems to be initialized at 1 in jump
    # note that ipopt will project the initial guess inside the bounds anyway.
    tf_init = 2000
    x_init =
        t -> [
            h_t0 + (t - t0) / (tf_init - t0) * (h_tf - h_t0),
            ϕ_t0,
            θ_t0,
            v_t0 + (t - t0) / (tf_init - t0) * (v_tf - v_t0),
            γ_t0 + (t - t0) / (tf_init - t0) * (γ_tf - γ_t0),
            ψ_t0,
        ]
    init = (state=x_init, control=[α_s, β_s], variable=[tf_init])

    # discretise the optimal control problem
    docp = direct_transcription(
        ocp,
        description...;
        lagrange_to_mayer=false,
        init=init,
        grid_size=grid_size,
        disc_method=:trapeze,
        kwargs...,
    )

    return docp
end
