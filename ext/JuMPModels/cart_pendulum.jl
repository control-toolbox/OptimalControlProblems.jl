"""
$(TYPEDSIGNATURES)

Constructs and returns a JuMP model for the **Cart–Pendulum Optimal Control Problem**.  
The objective is to swing a pendulum attached to a cart from the downward position to the upright position in the shortest possible time.  
The system dynamics, constraints, and objective are discretised over `N` steps.

# Arguments

- `::JuMPBackend`: Specifies the backend for building the JuMP model.
- `grid_size::Int=500`: (Keyword) Number of discretisation steps in the time grid.

# Returns

- `model::JuMP.Model`: A JuMP model representing the cart–pendulum optimal control problem.

# Example

```julia-repl
julia> using OptimalControlProblems
julia> using JuMP

julia> model = OptimalControlProblems.cart_pendulum(JuMPBackend(); N=200)
```

# References

- [Cart–Pendulum Optimal Control Problem](https://arxiv.org/pdf/2303.16746)
"""
function OptimalControlProblems.cart_pendulum(
    ::JuMPBackend,
    args...;
    grid_size::Int=grid_size_data(:cart_pendulum),
    parameters::Union{Nothing,NamedTuple}=nothing,
    kwargs...,
)

    # parameters
    params = parameters_data(:cart_pendulum, parameters)
    t0 = params[:t0]
    g = params[:g]
    L = params[:L]
    m = params[:m]
    I = m * L^2 / 12    # pendulum moment of inertia
    mcart = params[:mcart]
    Fex_l = params[:Fex_l]
    Fex_u = params[:Fex_u]
    x_l = params[:x_l]
    x_u = params[:x_u]
    v_l = params[:v_l]
    v_u = params[:v_u]
    tf_l = params[:tf_l]
    x_t0 = params[:x_t0]
    θ_t0 = params[:θ_t0]
    ω_t0 = params[:ω_t0]
    θ_tf = params[:θ_tf]
    ω_tf = params[:ω_tf]

    # model
    model = JuMP.Model(args...; kwargs...)

    # metadata: required
    model[:time_grid] = () -> range(t0, value(model[:tf]), grid_size+1) # tf is a free
    model[:state_components] = ["x", "v", "θ", "ω"]
    model[:costate_components] = ["∂x", "∂v", "∂θ", "∂ω"]
    model[:control_components] = ["Fex"]
    model[:variable_components] = ["tf", "ddx"]

    # N = grid_size
    @expression(model, N, grid_size)

    # variables and initial guess
    @variables(
        model,
        begin
            tf ≥ tf_l, (start = 1.0)
            ddx, (start = 0.1)
            x_l ≤ x[0:N] ≤ x_u, (start = 0.1)
            v_l ≤ v[0:N] ≤ v_u, (start = 0.1)
            θ[0:N], (start = 0.1)
            ω[0:N], (start = 0.1)
            Fex_l ≤ Fex[0:N] ≤ Fex_u, (start = 0.1)
        end
    )

    # boundary constraints
    @constraints(
        model,
        begin
            x[0] == x_t0
            θ[0] == θ_t0
            ω[0] == ω_t0
            θ[N] == θ_tf
            ω[N] == ω_tf
        end
    )

    # dynamics
    @expressions(
        model,
        begin
            Δt, (tf - t0) / N

            α_ddx[i = 0:N],
            1 / (I + 0.25 * m * L^2) * 0.5 * L * m * (-ddx * cos(θ[i]) - g * sin(θ[i]))

            ddCOG_1[i = 0:N], - L * sin(θ[i]) * ω[i] + L / 2 * cos(θ[i]) * α_ddx[i] + ddx
            ddCOG_2[i = 0:N], L * cos(θ[i]) * ω[i] + L / 2 * sin(θ[i]) * α_ddx[i]

            FXFY_1[i = 0:N], m * ddCOG_1[i]
            #FXFY_2[i=0:N], m * ddCOG_2[i] + m * g

            eq[i = 0:N], -FXFY_1[i] + Fex[i] - mcart * ddx

            J, mcart
            c[i = 0:N], eq[i] - J * ddx

            dx[i = 0:N], v[i]
            dv[i = 0:N], -1 / J * c[i]
            dθ[i = 0:N], ω[i]
            dω[i = 0:N],
            1 / (I + 0.25 * m * L^2) * 0.5 * L * m * (-dv[i] * cos(θ[i]) - g * sin(θ[i]))
        end
    )

    @constraints(
        model,
        begin
            ∂x[k = 1:N], x[k] == x[k - 1] + 0.5 * Δt * (dx[k] + dx[k - 1])
            ∂v[k = 1:N], v[k] == v[k - 1] + 0.5 * Δt * (dv[k] + dv[k - 1])
            ∂θ[k = 1:N], θ[k] == θ[k - 1] + 0.5 * Δt * (dθ[k] + dθ[k - 1])
            ∂ω[k = 1:N], ω[k] == ω[k - 1] + 0.5 * Δt * (dω[k] + dω[k - 1])
        end
    )

    # objective
    @objective(model, Min, tf)

    return model
end
