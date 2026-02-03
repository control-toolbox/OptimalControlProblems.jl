"""
$(TYPEDSIGNATURES)

Constructs the **Bioreactor optimal control problem** with **fixed initial conditions**.
"""
function OptimalControlProblems.bioreactor_s(
    ::OptimalControlBackend,
    description::Symbol...;
    grid_size::Int = grid_size_data(:bioreactor),
    parameters::Union{Nothing,NamedTuple} = nothing,
    kwargs...,
)
    # --- Parameters ---
    params = parameters_data(:bioreactor, parameters)
    t0, tf = params[:t0], params[:tf]
    β, c, γ = params[:β], params[:c], params[:γ]
    halfperiod = params[:halfperiod]
    Ks, μ2m, μbar, r = params[:Ks], params[:μ2m], params[:μbar], params[:r]

    # --- Model ---
    ocp = @def begin
        t ∈ [t0, tf], time
        x ∈ R⁴, state
        u ∈ R, control

        # Coordinate aliases
        y = x[1]
        s = x[2]
        b = x[3]
        k = x[4]

        # Path constraints
        y(t) ≥ 0
        s(t) ≥ 0
        b(t) ≥ 1e-3
        k(t) ≥ t0
        k(t) ≤ tf

        # Control bounds
        0 ≤ u(t) ≤ 1

        # Fixed Initial Conditions
        y(t0) == 0.05
        s(t0) == 0.5
        b(t0) == 0.5
        k(t0) == t0

        # Dynamics (Coordinatewise & Inline)
        
        # dy/dt
        ẋ[1](t) == (μbar * sin(k(t) * π / halfperiod)^2) * y(t) / (1 + y(t)) - (r + u(t)) * y(t)

        # ds/dt
        ẋ[2](t) == -(μ2m * s(t) / (s(t) + Ks)) * b(t) + u(t) * β * (γ * y(t) - s(t))

        # db/dt
        ẋ[3](t) == ((μ2m * s(t) / (s(t) + Ks)) - u(t) * β) * b(t)

        # dk/dt
        ẋ[4](t) == 1

        # Objective
        -∫((μ2m * s(t) / (s(t) + Ks)) * b(t) / (β + c)) → min
    end

    # --- Initial guess ---
    init = (state = [0.05, 0.5, 0.5, t0], control = 0.5)

    # --- Direct transcription ---
    docp = direct_transcription(
        ocp,
        description...;
        lagrange_to_mayer = false,
        init = init,
        grid_size = grid_size,
        disc_method = :trapeze,
        kwargs...,
    )

    return docp
end