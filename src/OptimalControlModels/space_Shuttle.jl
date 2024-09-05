"""
Space Shuttle Reentry Trajectory Problem:
    We want to find the optimal trajectory of a space shuttle reentry.
    The objective is to minimize the angle of attack at the terminal point.
    The problem is formulated as an OptimalControl model.
"""
function space_shuttle(;nh::Int=503)
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
    tf = 2009.0 # final time (sec)
    t0 = 0.0 # initial time (sec)

    @def ocp begin 
        ## parameters
        w = 203000.0  # weight (lb)
        g₀ = 32.174    # acceleration (ft/sec^2)
        m = w / g₀    # mass (slug)
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
        h_s = 2.6          # altitude (ft) / 1e5
        ϕ_s = deg2rad(0)   # longitude (rad)
        θ_s = deg2rad(0)   # latitude (rad)
        v_s = 2.56         # velocity (ft/sec) / 1e4
        γ_s = deg2rad(-1)  # flight path angle (rad)
        ψ_s = deg2rad(90)  # azimuth (rad)
        α_s = deg2rad(0)   # angle of attack (rad)
        β_s = deg2rad(0)   # bank angle (rad)
        t_s = 1.00 
        h_t = 0.8          # altitude (ft) / 1e5
        v_t = 0.25         # velocity (ft/sec) / 1e4
        γ_t = deg2rad(-5)  # flight path angle (rad)
        tf = 2012.0 # final time (sec)
        t0 = 0.0 # initial time (sec)

        ## define the problem
        t ∈ [ t0, tf ], time
        x ∈ R⁶, state
        u ∈ R², control

        ## state variables
        scaled_h = x₁
        ϕ = x₂
        θ = x₃
        scaled_v = x₄
        γ = x₅
        ψ = x₆

        ## control variables
        α = u₁
        β = u₂

        ## constraints
        # variable constraints
        # state constraints
        scaled_h(t) ≥ 0,                        (scaled_h_con)
        deg2rad(-89) ≤ θ(t) ≤ deg2rad(89),      (θ_con)
        scaled_v(t) ≥ 1e-4,                     (scaled_v_con)
        deg2rad(-89) ≤ γ(t) ≤ deg2rad(89),      (γ_con)
        # control constraints
        deg2rad(-89) ≤ β(t) ≤ deg2rad(1),       (β_con)
        deg2rad(-90) ≤ α(t) ≤ deg2rad(90),      (α_con)
        # initial conditions
        scaled_h(t0) == h_s,                    (scaled_h0_con)
        ϕ(t0) == ϕ_s,                           (ϕ0_con)
        θ(t0) == θ_s,                           (θ0_con)
        scaled_v(t0) == v_s,                    (scaled_v0_con)
        γ(t0) == γ_s,                           (γ0_con)
        ψ(t0) == ψ_s,                           (ψ0_con)
        # final conditions
        scaled_h(tf) == h_t,                    (scaled_hf_con)
        scaled_v(tf) == v_t,                    (scaled_vf_con)
        γ(tf) == γ_t,                           (γf_con)

        ## dynamics  
        ẋ(t) == dynamics(x(t),u(t))

        ## objective
        θ(tf) → max
    end

    ## dynamics
    function dynamics(x,u)
        scaled_h, ϕ, θ, scaled_v, γ, ψ = x
        α, β = u
        h = scaled_h * 1e5
        v = scaled_v * 1e4
        ## Helper functions
        c_D = b₀ + b₁ * rad2deg(α) + b₂ * (rad2deg(α)^2)
        c_L = a₀ + a₁ * rad2deg(α)
        ρ = ρ₀ * exp(-h/hᵣ)
        D = (1/2) * c_D * S * ρ * (v^2)
        L = (1/2) * c_L * S * ρ * (v^2)
        r = Rₑ + h
        g = μ / (r^2)

        ## dynamics  
        h_dot = v * sin(γ)
        ϕ_dot = (v/r) * cos(γ) * sin(ψ) / cos(θ)
        θ_dot = (v/r) * cos(γ) * cos(ψ)
        v_dot = -(D/m) - g*sin(γ)
        γ_dot = (L/(m*v)) * cos(β) + cos(γ) * ((v/r)-(g/v))
        ψ_dot = (1/(m*v*cos(γ))) * L*sin(β) + (v/(r*cos(θ))) * cos(γ) * sin(ψ) * sin(θ)

        return [ h_dot, ϕ_dot, θ_dot, v_dot, γ_dot, ψ_dot]
    end

    # Initial guess
    # Helper function for linear interpolation
    function linear_interpolate(x_s, x_t, nh)
        return [x_s + (i-1) / (n-1) * (x_t - x_s) for i in 1:nh]
    end
    # Interpolate each parameter separately
    h_interp = linear_interpolate(h_s, h_t, nh)
    ϕ_interp = linear_interpolate(ϕ_s, ϕ_s, nh) # no change in longitude
    θ_interp = linear_interpolate(θ_s, θ_s, nh) # no change in latitude
    v_interp = linear_interpolate(v_s, v_t, nh)
    γ_interp = linear_interpolate(γ_s, γ_t, nh)
    ψ_interp = linear_interpolate(ψ_s, ψ_s, nh) # no change in azimuth
    α_interp = linear_interpolate(α_s, α_s, nh) # no change in angle of attack
    β_interp = linear_interpolate(β_s, β_s, nh) # no change in bank angle
    t_interp = linear_interpolate(t_s, t_s, nh) # no change in time step
    # Combine all interpolated parameters into an array of arrays
    interpolated_values = [transpose([h, ϕ, θ, v, γ, ψ, α, β, t]) for (h, ϕ, θ, v, γ, ψ, α, β, t) in 
                            zip(h_interp, ϕ_interp, θ_interp, v_interp, γ_interp, ψ_interp, α_interp, β_interp, t_interp)]
    # Create the initial guess by summing the interpolated values
    initial_guess = reduce(vcat, interpolated_values)
    x_init = [initial_guess[i,1:6] for i in 1:nh];
    u_init = [initial_guess[i,7:8] for i in 1:nh];
    time_vec = LinRange(0.0,t_s*nh*4,nh)
    init = (time= time_vec, state= x_init, control= u_init)

    # NLPModel
    nlp = direct_transcription(ocp ,init = init, grid_size = nh)[2]

    return nlp
end

