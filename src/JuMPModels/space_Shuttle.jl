import Interpolations
"""
Space Shuttle Reentry Trajectory Problem:
    We want to find the optimal trajectory of a space shuttle reentry.
    The objective is to minimize the angle of attack at the terminal point.
    The problem is formulated as a JuMP model, and can be found [here](https://jump.dev/JuMP.jl/stable/tutorials/nonlinear/space_shuttle_reentry_trajectory/)
"""
function space_Shuttle(integration_rule::String = "rectangular";nh::Int64=503)
    ## Global variables
    w = 203000.0  # weight (lb)
    g₀ = 32.174    # acceleration (ft/sec^2)
    m = w / g₀    # mass (slug)

    ## Aerodynamic and atmospheric forces on the vehicle
    ρ₀ = 0.002378
    hᵣ = 23800.0
    Rₑ = 20902900.0
    μ = 0.14076539e17
    S = 2690.0
    a₀ = -0.20704
    a₁ = 0.029244
    b₀ = 0.07854
    b₁ = -0.61592e-2
    b₂ = 0.621408e-3
    c₀ = 1.0672181
    c₁ = -0.19213774e-1
    c₂ = 0.21286289e-3
    c₃ = -0.10117249e-5

    ## Initial conditions
    h_s = 2.6          # altitude (ft) / 1e5
    ϕ_s = deg2rad(0)   # longitude (rad)
    θ_s = deg2rad(0)   # latitude (rad)
    v_s = 2.56         # velocity (ft/sec) / 1e4
    γ_s = deg2rad(-1)  # flight path angle (rad)
    ψ_s = deg2rad(90)  # azimuth (rad)
    α_s = deg2rad(0)   # angle of attack (rad)
    β_s = deg2rad(0)   # bank angle (rad)
    t_s = 1.00         # time step (sec)

    ## Final conditions, the so-called Terminal Area Energy Management (TAEM)
    h_t = 0.8          # altitude (ft) / 1e5
    v_t = 0.25         # velocity (ft/sec) / 1e4
    γ_t = deg2rad(-5)  # flight path angle (rad)

    model = JuMP.Model()
    @variables(model, begin
        0 ≤ scaled_h[1:nh]                # altitude (ft) / 1e5
        ϕ[1:nh]                # longitude (rad)
        deg2rad(-89) ≤ θ[1:nh] ≤ deg2rad(89)  # latitude (rad)
        1e-4 ≤ scaled_v[1:nh]                # velocity (ft/sec) / 1e4
        deg2rad(-89) ≤ γ[1:nh] ≤ deg2rad(89)  # flight path angle (rad)
        ψ[1:nh]                # azimuth (rad)
        deg2rad(-90) ≤ α[1:nh] ≤ deg2rad(90)  # angle of attack (rad)
        deg2rad(-89) ≤ β[1:nh] ≤ deg2rad(1)  # bank angle (rad)
        3.5 ≤ Δt[1:(nh-1)] ≤ 4.5                 # time step (sec)
    end);

    ## Fix initial conditions
    # inial and final conditions
    @constraints(model,begin
        con_h0, scaled_h[1] == h_s
        con_ϕ0, ϕ[1] == ϕ_s
        con_θ0, θ[1] == θ_s
        con_v0, scaled_v[1] == v_s
        con_γ0, γ[1] == γ_s
        con_ψ0, ψ[1] == ψ_s
        con_hf, scaled_h[nh] == h_t
        con_vf, scaled_v[nh] == v_t
        con_γf, γ[nh] == γ_t
    end)

    ## Initial guess: linear interpolation between boundary conditions
    x_s = [h_s, ϕ_s, θ_s, v_s, γ_s, ψ_s, α_s, β_s,t_s]
    x_t = [h_t, ϕ_s, θ_s, v_t, γ_t, ψ_s, α_s, β_s,t_s]
    interp_linear = Interpolations.LinearInterpolation([1, nh], [x_s, x_t])
    initial_guess = mapreduce(transpose, vcat, interp_linear.(1:nh))
    set_start_value.(model[:scaled_h], vec(initial_guess[:,1]))
    set_start_value.(model[:ϕ], vec(initial_guess[:,2]))
    set_start_value.(model[:θ], vec(initial_guess[:,3]))
    set_start_value.(model[:scaled_v], vec(initial_guess[:,4]))
    set_start_value.(model[:γ], vec(initial_guess[:,5]))
    set_start_value.(model[:ψ], vec(initial_guess[:,6]))
    set_start_value.(model[:α], vec(initial_guess[:,7]))
    set_start_value.(model[:β], vec(initial_guess[:,8]))
    set_start_value.(model[:Δt], vec(initial_guess[1:end-1,9]))


    ## Functions to restore `h` and `v` to their true scale
    @expression(model, h[j = 1:nh], scaled_h[j] * 1e5)
    @expression(model, v[j = 1:nh], scaled_v[j] * 1e4)

    # Helper functions
    @expression(model, c_L[j = 1:nh], a₀ + a₁ * rad2deg(α[j]))
    @expression(model, c_D[j = 1:nh], b₀ + b₁ * rad2deg(α[j]) + b₂ * rad2deg(α[j])^2)
    @expression(model, ρ[j = 1:nh], ρ₀ * exp(-h[j] / hᵣ))
    @expression(model, D[j = 1:nh], 0.5 * c_D[j] * S * ρ[j] * v[j]^2)
    @expression(model, L[j = 1:nh], 0.5 * c_L[j] * S * ρ[j] * v[j]^2)
    @expression(model, r[j = 1:nh], Rₑ + h[j])
    @expression(model, g[j = 1:nh], μ / r[j]^2)

    ## Motion of the vehicle as a differential-algebraic system of equations (DAEs)
    @expression(model, δh[j = 1:nh], v[j] * sin(γ[j]))
    @expression(
        model,
        δϕ[j = 1:nh],
        (v[j] / r[j]) * cos(γ[j]) * sin(ψ[j]) / cos(θ[j])
    )
    @expression(model, δθ[j = 1:nh], (v[j] / r[j]) * cos(γ[j]) * cos(ψ[j]))
    @expression(model, δv[j = 1:nh], -(D[j] / m) - g[j] * sin(γ[j]))
    @expression(
        model,
        δγ[j = 1:nh],
        (L[j] / (m * v[j])) * cos(β[j]) +
        cos(γ[j]) * ((v[j] / r[j]) - (g[j] / v[j]))
    )
    @expression(
        model,
        δψ[j = 1:nh],
        (1 / (m * v[j] * cos(γ[j]))) * L[j] * sin(β[j]) +
        (v[j] / (r[j] * cos(θ[j]))) * cos(γ[j]) * sin(ψ[j]) * sin(θ[j])
    )

    if integration_rule == "rectangular"
        ## Rectangular integration
        @constraints(model,begin 
            con_dh[i=2:nh], h[i] == h[i-1] + Δt[i-1] * δh[i-1]
            con_dϕ[i=2:nh], ϕ[i] == ϕ[i-1] + Δt[i-1] * δϕ[i-1]
            con_dθ[i=2:nh], θ[i] == θ[i-1] + Δt[i-1] * δθ[i-1]
            con_dv[i=2:nh], v[i] == v[i-1] + Δt[i-1] * δv[i-1]
            con_dγ[i=2:nh], γ[i] == γ[i-1] + Δt[i-1] * δγ[i-1]
            con_dψ[i=2:nh], ψ[i] == ψ[i-1] + Δt[i-1] * δψ[i-1]
        end)
    elseif integration_rule == "trapezoidal"
        ## Trapezoidal integration
        @constraints(model,begin 
            con_dh[i=2:nh], h[i] == h[i-1] + 0.5 * Δt[i-1] * (δh[i-1] + δh[i]) 
            con_dϕ[i=2:nh], ϕ[i] == ϕ[i-1] + 0.5 * Δt[i-1] * (δϕ[i-1] + δϕ[i]) 
            con_dθ[i=2:nh], θ[i] == θ[i-1] + 0.5 * Δt[i-1] * (δθ[i-1] + δθ[i]) 
            con_dv[i=2:nh], v[i] == v[i-1] + 0.5 * Δt[i-1] * (δv[i-1] + δv[i]) 
            con_dγ[i=2:nh], γ[i] == γ[i-1] + 0.5 * Δt[i-1] * (δγ[i-1] + δγ[i]) 
            con_dψ[i=2:nh], ψ[i] == ψ[i-1] + 0.5 * Δt[i-1] * (δψ[i-1] + δψ[i]) 
        end)
    else
        @error "Unexpected integration rule '$(integration_rule)'"
    end

    @objective(model, Max, θ[n])

    return model
end