"""
$(TYPEDSIGNATURES)

Constructs and returns a JuMP model for the **Space Shuttle Reentry Trajectory Problem**.  
The model represents the dynamics of a space shuttle reentry with multiple states (altitude `h`, longitude `ϕ`, latitude `θ`, velocity `v`, flight path angle `γ`, azimuth `ψ`) and control inputs (angle of attack `α` and bank angle `β`).  
The objective is to maximise the terminal latitude (cross-range) while satisfying boundary conditions and vehicle dynamics.  
Note: no heating limit path constraint is included in this formulation.

# Arguments

- `::JuMPBackend`: Specifies the backend for building the JuMP model.
- `N::Int=500`: (Keyword) Number of discretisation steps for the time horizon.

# Returns

- `model::JuMP.Model`: A JuMP model representing the space shuttle reentry optimal control problem.

# Example

```julia-repl
julia> using OptimalControlProblems
julia> using JuMP

julia> model = OptimalControlProblems.space_shuttle(JuMPBackend(); N=200)
```

# References

- Problem formulation and tutorial available at: https://jump.dev/JuMP.jl/stable/tutorials/nonlinear/space_shuttle_reentry_trajectory/
"""
function OptimalControlProblems.space_shuttle(
    ::JuMPBackend, args...; N::Int=steps_number_data(:space_shuttle), 
    parameters::Union{Nothing, NamedTuple}=nothing,
    kwargs...
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
    model = JuMP.Model(args...; kwargs...)

    # ------------------------------------------------
    # expressions to get grid time infos
    @expressions(
        model,
        begin
            t0, t0  # (required if the initial time is fixed)
            N, N    # (required)
        end
    )
    # ------------------------------------------------

    # state, control and variable (final time)
    @variables(
        model,
        begin

            # state
            0 ≤ scaled_h[0:N]                          # altitude (ft) / scaling_h
            -2π ≤ ϕ[0:N] ≤ 2π                          # longitude (rad)
            deg2rad(-89) ≤ θ[0:N] ≤ deg2rad(89)        # latitude (rad)
            1e-4 ≤ scaled_v[0:N]                       # velocity (ft/sec) / scaling_v
            deg2rad(-89) ≤ γ[0:N] ≤ deg2rad(89)        # flight path angle (rad)
            -2π ≤ ψ[0:N] ≤ 2π                          # azimuth (rad)

            # control
            deg2rad(-90) ≤ α[0:N] ≤ deg2rad(90)        # angle of attack (rad)
            deg2rad(-89) ≤ β[0:N] ≤ deg2rad(1)         # bank angle (rad)

            #
            tf_min ≤ tf ≤ tf_max                        # final time (sec)
        end
    )

    ## Fix initial conditions
    # initial and final conditions
    @constraints(
        model,
        begin
            con_h0, scaled_h[0] == h_s
            con_ϕ0, ϕ[0] == ϕ_s
            con_θ0, θ[0] == θ_s
            con_v0, scaled_v[0] == v_s
            con_γ0, γ[0] == γ_s
            con_ψ0, ψ[0] == ψ_s
            con_hf, scaled_h[N] == h_t
            con_vf, scaled_v[N] == v_t
            con_γf, γ[N] == γ_t
        end
    )

    ## initial guess: linear interpolation between boundary conditions

    # Helper function for linear interpolation
    function linear_interpolate(x_s, x_t, n)
        return [x_s + (i - 1) / (n - 1) * (x_t - x_s) for i in 1:n]
    end

    # Interpolate each parameter separately
    h_interp = linear_interpolate(h_s, h_t, N+1)
    ϕ_interp = linear_interpolate(ϕ_s, ϕ_s, N+1) # no change in longitude
    θ_interp = linear_interpolate(θ_s, θ_s, N+1) # no change in latitude
    v_interp = linear_interpolate(v_s, v_t, N+1)
    γ_interp = linear_interpolate(γ_s, γ_t, N+1)
    ψ_interp = linear_interpolate(ψ_s, ψ_s, N+1) # no change in azimuth
    α_interp = linear_interpolate(α_s, α_s, N+1) # no change in angle of attack
    β_interp = linear_interpolate(β_s, β_s, N+1) # no change in bank angle
    t_interp = linear_interpolate(t_s, t_s, N+1) # no change in time step

    # Combine all interpolated parameters into an array of arrays
    interpolated_values = [
        transpose([h, ϕ, θ, v, γ, ψ, α, β, t]) for (h, ϕ, θ, v, γ, ψ, α, β, t) in zip(
            h_interp,
            ϕ_interp,
            θ_interp,
            v_interp,
            γ_interp,
            ψ_interp,
            α_interp,
            β_interp,
            t_interp,
        )
    ]

    # Create the initial guess by summing the interpolated values
    initial_guess = reduce(vcat, interpolated_values)
    set_start_value.(model[:scaled_h], vec(initial_guess[:, 1]))
    set_start_value.(model[:ϕ], vec(initial_guess[:, 2]))
    set_start_value.(model[:θ], vec(initial_guess[:, 3]))
    set_start_value.(model[:scaled_v], vec(initial_guess[:, 4]))
    set_start_value.(model[:γ], vec(initial_guess[:, 5]))
    set_start_value.(model[:ψ], vec(initial_guess[:, 6]))
    set_start_value.(model[:α], vec(initial_guess[:, 7]))
    set_start_value.(model[:β], vec(initial_guess[:, 8]))
    set_start_value.(model[:tf], (tf_min+tf_max)/2)
    #set_start_value.(model[:Δt], vec(initial_guess[1:(end - 1), 9]))

    ## Functions to restore `h` and `v` to their true scale
    @expression(model, h[j = 0:N], scaled_h[j] * scaling_h)
    @expression(model, v[j = 0:N], scaled_v[j] * scaling_v)

    # Helper functions
    @expression(model, c_L[j = 0:N], a₀ + a₁ * rad2deg(α[j]))
    @expression(model, c_D[j = 0:N], b₀ + b₁ * rad2deg(α[j]) + b₂ * rad2deg(α[j])^2)
    @expression(model, ρ[j = 0:N], ρ₀ * exp(-h[j] / hᵣ))
    @expression(model, D[j = 0:N], 0.5 * c_D[j] * S * ρ[j] * v[j]^2)
    @expression(model, L[j = 0:N], 0.5 * c_L[j] * S * ρ[j] * v[j]^2)
    @expression(model, r[j = 0:N], Rₑ + h[j])
    @expression(model, g[j = 0:N], μ / r[j]^2)

    ## Motion of the vehicle as a differential-algebraic system of equations (DAEs)
    @expression(model, δh[j = 0:N], v[j] * sin(γ[j]))
    @expression(model, δϕ[j = 0:N], (v[j] / r[j]) * cos(γ[j]) * sin(ψ[j]) / cos(θ[j]))
    @expression(model, δθ[j = 0:N], (v[j] / r[j]) * cos(γ[j]) * cos(ψ[j]))
    @expression(model, δv[j = 0:N], -(D[j] / m) - g[j] * sin(γ[j]))
    @expression(
        model,
        δγ[j = 0:N],
        (L[j] / (m * v[j])) * cos(β[j]) + cos(γ[j]) * ((v[j] / r[j]) - (g[j] / v[j]))
    )
    @expression(
        model,
        δψ[j = 0:N],
        (1 / (m * v[j] * cos(γ[j]))) * L[j] * sin(β[j]) +
            (v[j] / (r[j] * cos(θ[j]))) * cos(γ[j]) * sin(ψ[j]) * sin(θ[j])
    )

    @expression(model, Δt[i = 1:N], tf / N)

    @constraints(
        model,
        begin
            ∂h[i = 1:N], h[i] == h[i - 1] + 0.5 * Δt[i] * (δh[i - 1] + δh[i])
            ∂ϕ[i = 1:N], ϕ[i] == ϕ[i - 1] + 0.5 * Δt[i] * (δϕ[i - 1] + δϕ[i])
            ∂θ[i = 1:N], θ[i] == θ[i - 1] + 0.5 * Δt[i] * (δθ[i - 1] + δθ[i])
            ∂v[i = 1:N], v[i] == v[i - 1] + 0.5 * Δt[i] * (δv[i - 1] + δv[i])
            ∂γ[i = 1:N], γ[i] == γ[i - 1] + 0.5 * Δt[i] * (δγ[i - 1] + δγ[i])
            ∂ψ[i = 1:N], ψ[i] == ψ[i - 1] + 0.5 * Δt[i] * (δψ[i - 1] + δψ[i])
        end
    )

    @objective(model, Min, -θ[N])

    return model
end
