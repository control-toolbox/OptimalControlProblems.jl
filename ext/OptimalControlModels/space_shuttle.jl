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
function OptimalControlProblems.space_shuttle(
    ::OptimalControlBackend,
    description::Symbol...;
    N::Int=steps_number_data(:space_shuttle),
    parameters::Union{Nothing, NamedTuple}=nothing,
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

    ## 
    Δt_min = params[:Δt_min]
    Δt_max = params[:Δt_max]
    tf_min = N*Δt_min
    tf_max = N*Δt_max

    ## Initial conditions
    h_s = params[:h_s]
    ϕ_s = params[:ϕ_s]
    θ_s = params[:θ_s]
    v_s = params[:v_s]
    γ_s = params[:γ_s]
    ψ_s = params[:ψ_s]
    α_s = params[:α_s]
    β_s = params[:β_s]
    t_s = params[:t_s]

    ## Final conditions, the so-called Terminal Area Energy Management (TAEM)
    h_t = params[:h_t]
    v_t = params[:v_t]
    γ_t = params[:γ_t]

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
        -2π ≤ ϕ(t) ≤ 2π
        -2π ≤ ψ(t) ≤ 2π

        ## final time constraints
        tf_min ≤ tf ≤ tf_max

        ## state constraints
        0 ≤ scaled_h(t) ≤ Inf, (scaled_h_c)
        deg2rad(-89) ≤ θ(t) ≤ deg2rad(89), (θ_c)
        0 ≤ scaled_v(t) ≤ Inf, (scaled_v_c)
        deg2rad(-89) ≤ γ(t) ≤ deg2rad(89), (γ_c)

        ## control constraints
        deg2rad(-90) ≤ α(t) ≤ deg2rad(90), (α_c)
        deg2rad(-89) ≤ β(t) ≤ deg2rad(1), (β_c)

        ## initial conditions
        scaled_h(t0) == h_s, (scaled_h_i)
        ϕ(t0) == ϕ_s, (ϕ_i)
        θ(t0) == θ_s, (θ_i)
        scaled_v(t0) == v_s, (scaled_v_i)
        γ(t0) == γ_s, (γ_i)
        ψ(t0) == ψ_s, (ψ_i)

        ## final conditions
        scaled_h(tf) == h_t, (scaled_h_f)
        scaled_v(tf) == v_t, (scaled_v_f)
        γ(tf) == γ_t, (γ_f)

        # dynamics  
        ẋ(t) == dynamics(x(t), u(t))

        # objective
        -θ(tf) → min
    end

    # dynamics
    function dynamics(x, u)
        scaled_h, ϕ, θ, scaled_v, γ, ψ = x
        α, β = u
        h = scaled_h * scaling_h
        v = scaled_v * scaling_v

        ## Helper functions
        c_D = b₀ + b₁ * rad2deg(α) + b₂ * (rad2deg(α)^2)
        c_L = a₀ + a₁ * rad2deg(α)
        ρ = ρ₀ * exp(-h / hᵣ)
        D = (1 / 2) * c_D * S * ρ * (v^2)
        L = (1 / 2) * c_L * S * ρ * (v^2)
        r = Rₑ + h
        g = μ / (r^2)

        ## dynamics  
        h_dot = v * sin(γ)
        dϕ = (v / r) * cos(γ) * sin(ψ) / cos(θ)
        dθ = (v / r) * cos(γ) * cos(ψ)
        v_dot = -(D / m) - g * sin(γ)
        γ_dot = (L / (m * v)) * cos(β) + cos(γ) * ((v / r) - (g / v))
        ψ_dot =
            (1 / (m * v * cos(γ))) * L * sin(β) +
            (v / (r * cos(θ))) * cos(γ) * sin(ψ) * sin(θ)

        return [h_dot / scaling_h, dϕ, dθ, v_dot / scaling_v, γ_dot, ψ_dot]
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
