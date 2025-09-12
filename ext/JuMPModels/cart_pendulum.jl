"""
$(TYPEDSIGNATURES)

Constructs and returns a JuMP model for the **Cart–Pendulum Optimal Control Problem**.  
The objective is to swing a pendulum attached to a cart from the downward position to the upright position in the shortest possible time.  
The system dynamics, constraints, and objective are discretised over `N` steps.

# Arguments

- `::JuMPBackend`: Specifies the backend for building the JuMP model.
- `N::Int=500`: (Keyword) Number of discretisation steps in the time grid.

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
    ::JuMPBackend, args...; N::Int=steps_number_data(:cart_pendulum), 
    parameters::Union{Nothing, NamedTuple}=nothing,
    kwargs...
)

    # parameters
    params = parameters_data(:cart_pendulum, parameters)
    t0 = params[:t0]
    g = params[:g]
    L = params[:L]
    m = params[:m]
    I = m * L^2 / 12    # pendulum moment of inertia
    mcart = params[:mcart]
    max_f = params[:max_f]
    max_x = params[:max_x]
    max_v = params[:max_v]

    # model
    model = JuMP.Model(args...; kwargs...)

    # ------------------------------------------------
    # expressions to get grid time infos
    @expressions(
        model,
        begin
            t0, t0  # (required if the initial time is fixed)
            N, N    # (required)
        end
    )
    # ------------------------------------------------

    # variables and initial guess
    @variables(
        model,
        begin
            0.1 <= tf, (start = 1.0)
            ddx, (start = 0.1)
            -max_x <= x[0:N] <= max_x, (start = 0.1)
            -max_v <= v[0:N] <= max_v, (start = 0.1)
            θ[0:N], (start = 0.1)
            ω[0:N], (start = 0.1)
            -max_f <= Fex[0:N] <= max_f, (start = 0.1)
        end
    )

    # boundary constraints
    @constraints(
        model,
        begin
            x[0] == 0
            θ[0] == 0
            ω[0] == 0
            θ[N] == π
            ω[N] == 0
        end
    )

    # dynamics
    @expressions(
        model,
        begin
            step, tf / N

            α_ddx[i = 0:N],
            1 / (I + 0.25 * m * L^2) * 0.5 * L * m * (-ddx * cos(θ[i]) - g * sin(θ[i]))

            ddCOG_1[i = 0:N], - L * sin(θ[i]) * ω[i] + L / 2 * cos(θ[i]) * α_ddx[i] + ddx
            ddCOG_2[i = 0:N], L * cos(θ[i]) * ω[i] + L / 2 * sin(θ[i]) * α_ddx[i]

            FXFY_1[i = 0:N], m * ddCOG_1[i]
            #FXFY_2[i=0:N], m * ddCOG_2[i] + m * g

            eq[i = 0:N], -FXFY_1[i] + Fex[i] - mcart * ddx

            J, mcart
            c[i = 0:N], eq[i] - J * ddx

            dv[i = 0:N], -1 / J * c[i]
            dω[i = 0:N],
            1 / (I + 0.25 * m * L^2) * 0.5 * L * m * (-dv[i] * cos(θ[i]) - g * sin(θ[i]))
        end
    )

    @constraints(
        model,
        begin
            ∂x[k = 1:N], x[k] == x[k - 1] + 0.5 * step * (v[k] + v[k - 1])
            ∂v[k = 1:N], v[k] == v[k - 1] + 0.5 * step * (dv[k] + dv[k - 1])
            ∂θ[k = 1:N], θ[k] == θ[k - 1] + 0.5 * step * (ω[k] + ω[k - 1])
            ∂ω[k = 1:N], ω[k] == ω[k - 1] + 0.5 * step * (dω[k] + dω[k - 1])
        end
    )

    # objective
    @objective(model, Min, tf)

    return model
end
