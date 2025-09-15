"""
$(TYPEDSIGNATURES)

Constructs and returns a JuMP model for the **Insurance Optimal Control Problem**.  
The model represents a simplified insurance management scenario where the objective is to optimise the utility function `U` over time, subject to capital accumulation dynamics and other constraints.  
The system is discretised using `N` steps, and collocation constraints enforce the dynamics of the states `I`, `m`, and `x₃`.

# Arguments

- `::JuMPBackend`: Specifies the backend for building the JuMP model.
- `grid_size::Int=500`: (Keyword) Number of discretisation steps in the time grid.

# Returns

- `model::JuMP.Model`: A JuMP model representing the insurance optimal control problem.

# Example

```julia-repl
julia> using OptimalControlProblems
julia> using JuMP

julia> model = OptimalControlProblems.insurance(JuMPBackend(); N=100)
```

# References

- Problem formulation available at: https://github.com/control-toolbox/bocop/tree/main/bocop
"""
function OptimalControlProblems.insurance(
    ::JuMPBackend, args...; grid_size::Int=grid_size_data(:insurance), 
    parameters::Union{Nothing, NamedTuple}=nothing,
    kwargs...
)

    # parameters
    params = parameters_data(:insurance, parameters)
    t0 = params[:t0]
    tf = params[:tf]
    γ = params[:γ]
    λ = params[:λ]
    h0 = params[:h0]
    w = params[:w]
    s = params[:s]
    k = params[:k]
    σ = params[:σ]
    α = params[:α]
    I_l = params[:I_l]
    I_u = params[:I_u]
    m_l = params[:m_l]
    m_u = params[:m_u]
    h_l = params[:h_l]
    h_u = params[:h_u]
    R_l = params[:R_l]
    H_l = params[:H_l]
    U_l = params[:U_l]
    dUdR_l = params[:dUdR_l]
    P_l = params[:P_l]
    I_t0 = params[:I_t0]
    m_t0 = params[:m_t0]
    x₃_t0 = params[:x₃_t0]
    
    # model
    model = JuMP.Model(args...; kwargs...)

    # ------------------------------------------------
    # expressions to get grid time infos
    @expressions(
        model,
        begin
            t0, t0  # (required if the initial time is fixed)
            tf, tf  # (required if the final time is fixed)
            N, grid_size    # (required)
        end
    )
    # ------------------------------------------------

    # state, control and initial guess
    @variables(
        model,
        begin
            I_l ≤ I[0:N] ≤ I_u,     (start = 0.1)
            m_l ≤ m[0:N] ≤ m_u,     (start = 0.1)
            x₃[0:N],                  (start = 0.1)
            h_l ≤ h[0:N] ≤ h_u,     (start = 0.1)
            R[0:N] ≥ R_l,            (start = 0.1)
            H[0:N] ≥ H_l,            (start = 0.1)
            U[0:N] ≥ U_l,            (start = 0.1)
            dUdR[0:N] ≥ dUdR_l,      (start = 0.1)
            P ≥ P_l,                 (start = 0.1)
        end
    )

    # boundary constraints  
    @constraints(
        model,
        begin
            I[0] == I_t0
            m[0] == m_t0
            x₃[0] == x₃_t0
            P - x₃[N] == 0
        end
    )

    @expressions(
        model,
        begin
            Δt, (tf - t0) / N
            t[i = 0:N], t0 + i * Δt
            ε[i = 0:N], k * (t[i] - t0) / (tf - t[i] + 1)
            fx[i = 0:N], λ * exp(-λ * (t[i] - t0)) + exp(-λ * (tf - t0)) / (tf - t0)
            v[i = 0:N], m[i]^(α / 2) / (1 + m[i]^(α / 2))
            vprime[i = 0:N], α / 2 * m[i]^(α / 2 - 1) / (1 + m[i]^(α / 2))^2
        end
    )

    @constraints(
        model,
        begin
            cond1[i = 0:N], R[i] - (w - P + I[i] - m[i] - ε[i]) == 0
            cond2[i = 0:N], H[i] - (h0 - γ * i * Δt * (1 - v[i])) == 0
            cond3[i = 0:N], U[i] - (1 - exp(-s * R[i]) + H[i]) == 0
            cond4[i = 0:N], dUdR[i] - (s * exp(-s * R[i])) == 0
        end
    )

    # dynamics
    @expressions(
        model,
        begin
            # dynamics
            dI[i = 0:N], (1 - γ * (t[i] - t0) * vprime[i] / dUdR[i]) * h[i]
            dm[i = 0:N], h[i]
            dx₃[i = 0:N], (1 + σ) * I[i] * fx[i]

            # objective
            dc[i = 0:N], U[i] * fx[i]
        end
    )

    @constraints(
        model,
        begin
            ∂I[i = 1:N], I[i] == I[i - 1] + 0.5 * Δt * (dI[i] + dI[i - 1])
            ∂m[i = 1:N], m[i] == m[i - 1] + 0.5 * Δt * (dm[i] + dm[i - 1])
            ∂x₃[i = 1:N], x₃[i] == x₃[i - 1] + 0.5 * Δt * (dx₃[i] + dx₃[i - 1])
        end
    )

    # objective
    @objective(model, Max, 0.5 * Δt * sum(dc[i] + dc[i - 1] for i in 1:N))

    return model
end
