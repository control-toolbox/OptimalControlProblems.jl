"""
$(TYPEDSIGNATURES)

Constructs an OptimalControl problem representing the Bioreactor problem.
Defines state/control variables, boundary conditions, dynamics, and objective.
Performs direct transcription to produce a DOCP model.
"""

function OptimalControlProblems.bioreactor(
    ::OptimalControlBackend,
    description::Symbol...;
    grid_size::Int=grid_size_data(:bioreactor),
    parameters::Union{Nothing,NamedTuple}=nothing,
    kwargs...,
)

    # --- 1. Parameters ---
    params = parameters_data(:bioreactor, parameters)

    t0, tf = params[:t0], params[:tf]
    β, c, γ = params[:β], params[:c], params[:γ]
    halfperiod = params[:halfperiod]
    Ks, μ2m, μbar, r = params[:Ks], params[:μ2m], params[:μbar], params[:r]

    y_l, s_l, b_l = params[:y_l], params[:s_l], params[:b_l]
    u_l, u_u = params[:u_l], params[:u_u]

    y_t0_l, y_t0_u = params[:y_t0_l], params[:y_t0_u]
    s_t0_l, s_t0_u = params[:s_t0_l], params[:s_t0_u]
    b_t0_l, b_t0_u = params[:b_t0_l], params[:b_t0_u]

    # --- 2. Model definition ---
    ocp = @def begin
        t ∈ [t0, tf], time

        # Named state variables
        x = (y, s, b, k) ∈ R⁴, state

        # Control
        u ∈ R, control

        # Path constraints
        y(t) ≥ 0
        s(t) ≥ 0
        b(t) ≥ 1e-3
        k(t) ≥ t0
        k(t) ≤ tf

        # Control bounds
        u_l ≤ u(t) ≤ u_u

        # Initial bounds
        y_t0_l ≤ y(t0) ≤ y_t0_u
        s_t0_l ≤ s(t0) ≤ s_t0_u
        b_t0_l ≤ b(t0) ≤ b_t0_u
        k(t0) == t0

        # Smooth light growth term (AD-friendly)
        light = sin(k(t) * π / halfperiod)^2

        # Dynamics (named derivatives required by DSL)
        ẏ(t) == (μbar * light) * y(t) / (1 + y(t)) - (r + u(t)) * y(t)

        ṡ(t) == -(μ2m * s(t) / (s(t) + Ks)) * b(t) +
                 u(t) * β * (γ * y(t) - s(t))

        ḃ(t) == ((μ2m * s(t) / (s(t) + Ks)) - u(t) * β) * b(t)

        ḱ(t) == 1

        # Objective
        -∫((μ2m * s(t) / (s(t) + Ks)) * b(t) / (β + c)) → min
    end

    # --- 3. Initial guess ---
    init = (
        state = [0.15, 2.75, 1.75, t0],
        control = 0.5,
    )

    # --- 4. Direct transcription ---
    docp = direct_transcription(
        ocp,
        description...;
        lagrange_to_mayer = false,
        init = init,
        grid_size = grid_size,
        disc_method = :trapezoid,
        kwargs...,
    )

    return docp
end
