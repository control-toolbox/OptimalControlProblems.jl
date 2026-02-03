"""
$(TYPEDSIGNATURES)

Constructs an OptimalControl problem representing the Bioreactor problem
with fixed initial conditions.

This version fixes the initial state and enforces a biomass safety bound.
Direct transcription is applied to build the DOCP.
"""

function OptimalControlProblems.bioreactor_s(
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
        0 ≤ u(t) ≤ 1

        # Fixed initial conditions
        y(t0) == 0.05
        s(t0) == 0.5
        b(t0) == 0.5
        k(t0) == t0

        # Smooth light term (AD-safe)
        light = sin(k(t) * π / halfperiod)^2

        # Dynamics using named derivatives
        ẏ(t) == (μbar * light) * y(t) / (1 + y(t)) - (r + u(t)) * y(t)

        ṡ(t) == -(μ2m * s(t) / (s(t) + Ks)) * b(t) +
                 u(t) * β * (γ * y(t) - s(t))

        ḃ(t) == ((μ2m * s(t) / (s(t) + Ks)) - u(t) * β) * b(t)

        ḱ(t) == 1

        # Objective functional
        -∫((μ2m * s(t) / (s(t) + Ks)) * b(t) / (β + c)) → min
    end

    # --- 3. Initial guess ---
    init = (
        state = [0.05, 0.5, 0.5, t0],
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
