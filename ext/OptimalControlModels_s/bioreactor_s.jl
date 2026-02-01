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
    # --- 1. Paramètres ---
    params = parameters_data(:bioreactor, parameters)
    t0, tf = params[:t0], params[:tf]
    β, c, γ = params[:β], params[:c], params[:γ]
    halfperiod = params[:halfperiod]
    Ks, μ2m, μbar, r = params[:Ks], params[:μ2m], params[:μbar], params[:r]

    # --- 2. Fonctions Utilitaires (AVANT @def) ---
    function growth(s, μ2m, Ks)
        return μ2m * s / (s + Ks)
    end

    function light(time, halfperiod)
        days = time / (halfperiod * 2)
        tau = (days - floor(days)) * 2π
        return max(0, sin(tau))^2
    end

    # --- 3. Modèle ---
    ocp = @def begin
        t ∈ [t0, tf], time
        x = (y, s, b) ∈ R³, state
        u ∈ R, control

        # Contraintes bornes
        x(t) ≥ [0, 0, 1e-3]
        0 ≤ u(t) ≤ 1

        # Conditions initiales FIXES
        x(t0) == [0.05, 0.5, 0.5]

        # Dynamique (Calculs "inline")
        ẋ[1](t) == (light(t, halfperiod) * μbar) * y(t) / (1 + y(t)) - (r + u(t)) * y(t)
        ẋ[2](t) == -growth(s(t), μ2m, Ks) * b(t) + u(t) * β * (γ * y(t) - s(t))
        ẋ[3](t) == (growth(s(t), μ2m, Ks) - u(t) * β) * b(t)

        # Objectif
        -∫(growth(s(t), μ2m, Ks) * b(t) / (β + c)) → min
    end

    # --- 4. Transcription ---
    init = (state=[0.05, 0.5, 0.5], control=0.5)

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