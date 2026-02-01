"""
$(TYPEDSIGNATURES)

Constructs an **OptimalControl problem** representing the **Bioreactor problem** with **fixed initial conditions**.
The function sets up the state variables (algae , substrate , biomass ), the control variable (flow rate ), and the specific dynamics governing the photobioreactor-digester coupling.

Unlike the general case, this version (`_s`) fixes the initial state to a specific point x(0) = [0.05, 0.5, 0.5] and enforces a safety constraint b(t) ge 10^{-3} to prevent biomass washout.

It performs direct transcription to generate a discrete optimal control problem (DOCP).

# Arguments

- `::OptimalControlBackend`: Placeholder type specifying the OptimalControl backend or solver interface.
- `grid_size::Int=grid_size_data(:bioreactor)`: (Keyword) Number of discretisation points for the direct transcription grid.

# Returns

- `docp`: The direct optimal control problem object representing the discretised Bioreactor problem.

# Example

```julia-repl
julia> using OptimalControlProblems

julia> docp = OptimalControlProblems.bioreactor(OptimalControlBackend(); N=100);
```
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

    w = π / halfperiod

    # --- 2. Model ---
    ocp = @def begin
        t ∈ [t0, tf], time
        x = (y, s, b, k) ∈ R⁴, state
        u ∈ R, control

        # Constraints
        x(t) ≥ [0, 0, 1e-3, t0]
        x(t) ≤ [Inf, Inf, Inf, tf]
        0 ≤ u(t) ≤ 1

        # Fixed Initial Conditions
        x(t0) == [0.05, 0.5, 0.5, t0]

        # Dynamics (Using k(t) instead of t for explicit time)
        ẋ[1](t) == (μbar * max(0, sin(k(t) * w))^2) * y(t) / (1 + y(t)) - (r + u(t)) * y(t)
        ẋ[2](t) == -(μ2m * s(t) / (s(t) + Ks)) * b(t) + u(t) * β * (γ * y(t) - s(t))
        ẋ[3](t) == ((μ2m * s(t) / (s(t) + Ks)) - u(t) * β) * b(t)
        ẋ[4](t) == 1

        # Objective
        -∫((μ2m * s(t) / (s(t) + Ks)) * b(t) / (β + c)) → min
    end

    # --- 3. Transcription ---
    init = (state=[0.05, 0.5, 0.5, t0], control=0.5)

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