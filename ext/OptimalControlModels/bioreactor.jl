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

    # Pre-calculate frequency to simplify the equation inside the macro
    w = π / halfperiod

    # --- 2. Model ---
    ocp = @def begin
        t ∈ [t0, tf], time
     
        x = (y, s, b, k) ∈ R⁴, state
        u ∈ R, control

        # Constraints
   
        x(t) ≥ [0, 0, 1e-3, t0]
        x(t) ≤ [Inf, Inf, Inf, tf] 
        u_l ≤ u(t) ≤ u_u
        
       
        [y_t0_l, s_t0_l, b_t0_l, t0] ≤ x(t0) ≤ [y_t0_u, s_t0_u, b_t0_u, t0]

        # Dynamics
 
        
        # dy/dt
        ẋ[1](t) == (μbar * max(0, sin(k(t) * w))^2) * y(t) / (1 + y(t)) - (r + u(t)) * y(t)
        
        # ds/dt
        ẋ[2](t) == -(μ2m * s(t) / (s(t) + Ks)) * b(t) + u(t) * β * (γ * y(t) - s(t))
        
        # db/dt
        ẋ[3](t) == ((μ2m * s(t) / (s(t) + Ks)) - u(t) * β) * b(t)

       
        ẋ[4](t) == 1

    
        -∫((μ2m * s(t) / (s(t) + Ks)) * b(t) / (β + c)) → min
    end

    init = (state=[0.15, 2.75, 1.75, t0], control=0.5)

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