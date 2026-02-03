"""
Constructs an OptimalControl bioreactor model.
Vector state form is required by CTParser.
"""

function OptimalControlProblems.bioreactor(
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

    y_l, s_l, b_l = params[:y_l], params[:s_l], params[:b_l]
    u_l, u_u = params[:u_l], params[:u_u]

    y_t0_l, y_t0_u = params[:y_t0_l], params[:y_t0_u]
    s_t0_l, s_t0_u = params[:s_t0_l], params[:s_t0_u]
    b_t0_l, b_t0_u = params[:b_t0_l], params[:b_t0_u]

    # --- Model ---
    ocp = @def begin
        t ∈ [t0, tf], time

        # Vector state required by parser
        x ∈ R⁴, state
        u ∈ R, control

        # State aliases for readability
        y = x[1]
        s = x[2]
        b = x[3]
        k = x[4]

        # Constraints
        y(t) ≥ 0
        s(t) ≥ 0
        b(t) ≥ 1e-3
        k(t) ≥ t0
        k(t) ≤ tf

        u_l ≤ u(t) ≤ u_u

        y_t0_l ≤ y(t0) ≤ y_t0_u
        s_t0_l ≤ s(t0) ≤ s_t0_u
        b_t0_l ≤ b(t0) ≤ b_t0_u
        k(t0) == t0

        # Smooth light term
        light = sin(k(t) * π / halfperiod)^2

        # Vector dynamics (required form)
        ẋ(t) == [
            (μbar * light) * y(t) / (1 + y(t)) - (r + u(t)) * y(t),
            -(μ2m * s(t) / (s(t) + Ks)) * b(t) + u(t) * β * (γ * y(t) - s(t)),
            ((μ2m * s(t) / (s(t) + Ks)) - u(t) * β) * b(t),
            1
        ]

        # Objective
        -∫((μ2m * s(t) / (s(t) + Ks)) * b(t) / (β + c)) → min
    end

    # --- Initial guess ---
    init = (state = [0.15, 2.75, 1.75, t0], control = 0.5)

    # --- Transcription ---
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
