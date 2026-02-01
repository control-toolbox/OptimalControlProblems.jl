"""
$(TYPEDSIGNATURES)

Constructs an **OptimalControl problem** representing the Bioreactor problem.  
The function defines the state and control variables, boundary conditions, path constraints, growth and light dynamics, and an objective functional.  
It performs direct transcription to create a discretised optimal control problem (DOCP) and the corresponding nonlinear programming (NLP) model.

# Arguments

- `::OptimalControlBackend`: Placeholder type specifying the OptimalControl backend or solver interface.
- `grid_size::Int=500`: (Keyword) Number of discretisation points for the direct transcription grid.

# Returns

- `docp`: The direct optimal control problem object representing the discretised Bioreactor problem.
- `nlp`: The corresponding nonlinear programming model generated from the DOCP, ready for solver input.

# Example

```julia-repl
julia> using OptimalControlProblems

julia> docp = OptimalControlProblems.bioreactor(OptimalControlBackend(); N=100);
```

# References

- BOCOP repository: https://github.com/control-toolbox/bocop/tree/main/bocop
"""

# --- HELPER FUNCTIONS (Must be defined outside the main function) ---

# Growth model (Monod)
function _bio_growth(s, μ2m, Ks)
    return μ2m * s / (s + Ks)
end

# Light model
function _bio_light(t, halfperiod, μbar)
    # Explicit periodicity
    w = π / halfperiod
    return max(0, sin(w * t))^2 * μbar
end


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

    # --- 2. Model ---
    ocp = @def begin
        t ∈ [t0, tf], time
        x = (y, s, b) ∈ R³, state
        u ∈ R, control

        # Box constraints (including safety constraint b >= 1e-3)
        x(t) ≥ [0, 0, 1e-3]
        u_l ≤ u(t) ≤ u_u
        
        # Initial conditions (Inequalities)
        [y_t0_l, s_t0_l, b_t0_l] ≤ x(t0) ≤ [y_t0_u, s_t0_u, b_t0_u]

        # Dynamics
        # Using external helper functions to avoid scope errors
        ẋ[1](t) == _bio_light(t, halfperiod, μbar) * y(t) / (1 + y(t)) - (r + u(t)) * y(t)
        ẋ[2](t) == -_bio_growth(s(t), μ2m, Ks) * b(t) + u(t) * β * (γ * y(t) - s(t))
        ẋ[3](t) == (_bio_growth(s(t), μ2m, Ks) - u(t) * β) * b(t)

        # Objective: Minimize negative integral (Maximize production)
        -∫(_bio_growth(s(t), μ2m, Ks) * b(t) / (β + c)) → min
    end

    # --- 3. Transcription ---
    init = (state=[0.15, 2.75, 1.75], control=0.5)

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