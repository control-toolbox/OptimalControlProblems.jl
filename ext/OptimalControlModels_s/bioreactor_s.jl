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
    # Retrieve physical constants
    t0 = params[:t0]
    tf = params[:tf]
    β, c, γ = params[:β], params[:c], params[:γ]
    halfperiod = params[:halfperiod]
    Ks, μ2m, μbar, r = params[:Ks], params[:μ2m], params[:μbar], params[:r]

    # --- 3. Auxiliary functions ---
    function growth(s, μ2m, Ks)
        return μ2m * s / (s + Ks)
    end

    function light(time, halfperiod)
        days = time / (halfperiod * 2)
        tau = (days - floor(days)) * 2π
        return max(0, sin(tau))^2
    end

    # --- 2. The Model ---
    ocp = @def begin
        t ∈ [t0, tf], time
        x = (y, s, b) ∈ R³, state
        u ∈ R, control

        # Simple bound constraints (Step B: discrete method)
        # Ensure non-negativity and biomass survival
        y(t) ≥ 0
        s(t) ≥ 0
        b(t) ≥ 0.001  # The safety constraint
        0 ≤ u(t) ≤ 1

        # FIXED INITIAL CONDITIONS (The "_s" suffix)
        # Fixing start exactly at lower bounds of the original problem
        x(t0) == [0.05, 0.5, 0.5] 

        # Dynamics (Light + Growth)
        μ = light(t, halfperiod) * μbar
        μ2 = growth(s(t), μ2m, Ks)

        ẋ[1](t) == μ * y(t) / (1 + y(t)) - (r + u(t)) * y(t)
        ẋ[2](t) == -μ2 * b(t) + u(t) * β * (γ * y(t) - s(t))
        ẋ[3](t) == (μ2 - u(t) * β) * b(t)

        # Objective: Maximize methane (Minimize the opposite)
        -∫(μ2 * b(t) / (β + c)) → min
    end

    # --- 4. Initialization (CRITICAL) ---
    # Start with constant state equal to initial point
    # and average control (0.5) to help the solver
    init = (state=[0.05, 0.5, 0.5], control=0.5)

    # --- 5. Transcription ---
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