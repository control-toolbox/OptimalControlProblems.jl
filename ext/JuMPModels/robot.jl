"""
$(TYPEDSIGNATURES)

Constructs and returns a JuMP model for the **Robot Arm Optimal Control Problem**.  
The model represents a robot arm with three states (ρ, θ, ϕ) controlled by three inputs (uρ, uθ, uϕ).  
The objective is to minimise the final time required for the robot arm to move between specified initial and final positions while respecting dynamic and boundary constraints.

# Arguments

- `::JuMPBackend`: Specifies the backend for building the JuMP model.
- `grid_size::Int=250`: (Keyword) Number of discretisation steps for the time horizon.

# Returns

- `model::JuMP.Model`: A JuMP model representing the robot arm optimal control problem.

# Example

```julia-repl
julia> using OptimalControlProblems
julia> using JuMP

julia> model = OptimalControlProblems.robot(JuMPBackend(); N=100)
```

# References

- Problem formulation available at: https://github.com/MadNLP/COPSBenchmark.jl/blob/main/src/robot.jl
"""
function OptimalControlProblems.robot(
    ::JuMPBackend, args...; grid_size::Int=grid_size_data(:robot), 
    parameters::Union{Nothing, NamedTuple}=nothing,
    kwargs...
)

    # parameters
    params = parameters_data(:robot, parameters)
    t0 = params[:t0]

    # total length of arm
    L = params[:L]

    # Upper bounds on the controls
    uρ_l = params[:uρ_l]
    uρ_u = params[:uρ_u]
    uθ_l = params[:uθ_l]
    uθ_u = params[:uθ_u]
    uϕ_l = params[:uϕ_l]
    uϕ_u = params[:uϕ_u]

    # Initial positions of the length and the angles for the robot arm
    ρ_t0 = params[:ρ_t0]
    θ_t0 = params[:θ_t0]
    ϕ_t0 = params[:ϕ_t0]
    dρ_t0 = params[:dρ_t0]
    dθ_t0 = params[:dθ_t0]
    dϕ_t0 = params[:dϕ_t0]

    # Final positions
    ρ_tf = params[:ρ_tf]
    θ_tf = params[:θ_tf]
    ϕ_tf = params[:ϕ_tf]
    dρ_tf = params[:dρ_tf]
    dθ_tf = params[:dθ_tf]
    dϕ_tf = params[:dϕ_tf]

    #
    ρ_l = params[:ρ_l]
    ρ_u = L
    θ_l = params[:θ_l]
    θ_u = params[:θ_u]
    ϕ_l = params[:ϕ_l]
    ϕ_u = params[:ϕ_u]
    tf_l = params[:tf_l]

    # model
    model = JuMP.Model(args...; kwargs...)

    # ------------------------------------------------
    # expressions to get grid time infos
    @expressions(
        model,
        begin
            t0, t0  # (required if the initial time is fixed)
            N, grid_size    # (required)
        end
    )
    # ------------------------------------------------

    # state, control, variable (final time) and initial guess
    @variables(
        model,
        begin
            ρ_l ≤ ρ[k = 0:N] ≤ ρ_u,         (start = ρ_t0)
            θ_l ≤ θ[k = 0:N] ≤ θ_u,         (start = 2π/3 * (k / N)^2)
            ϕ_l ≤ ϕ[k = 0:N] ≤ ϕ_u,         (start = ϕ_t0)

            dρ[k = 0:N],                    (start = 0)
            dθ[k = 0:N],                    (start = 4π/3 * (k / N))
            dϕ[k = 0:N],                    (start = 0)

            uρ_l ≤ uρ[0:N] ≤ uρ_u,          (start = 0)
            uθ_l ≤ uθ[0:N] ≤ uθ_u,          (start = 0)
            uϕ_l ≤ uϕ[0:N] ≤ uϕ_u,          (start = 0)

            tf ≥ tf_l,                      (start = 1)
        end
    )

    # Boundary condition
    @constraints(
        model,
        begin
            # initial
            ρ[0] == ρ_t0
            θ[0] == θ_t0
            ϕ[0] == ϕ_t0
            dρ[0] == dρ_t0
            dθ[0] == dθ_t0
            dϕ[0] == dϕ_t0

            # final
            ρ[N] == ρ_tf
            θ[N] == θ_tf
            ϕ[N] == ϕ_tf
            dρ[N] == dρ_tf
            dθ[N] == dθ_tf
            dϕ[N] == dϕ_tf
        end
    )

    # dynamics
    @expressions(
        model,
        begin
            #
            Δt, (tf - t0) / N

            #
            I_θ[i = 0:N], ((L - ρ[i])^3 + ρ[i]^3) * (sin(ϕ[i]))^2
            I_ϕ[i = 0:N], (L - ρ[i])^3 + ρ[i]^3

            #
            ddρ[i = 0:N], uρ[i] / L
            ddθ[i = 0:N], 3 * uθ[i] / I_θ[i]
            ddϕ[i = 0:N], 3 * uϕ[i] / I_ϕ[i]
        end
    )

    @constraints(
        model,
        begin
            ∂ρ[i = 1:N], ρ[i] == ρ[i - 1] + 0.5 * Δt * (dρ[i] + dρ[i - 1])
            ∂ϕ[i = 1:N], ϕ[i] == ϕ[i - 1] + 0.5 * Δt * (dϕ[i] + dϕ[i - 1])
            ∂θ[i = 1:N], θ[i] == θ[i - 1] + 0.5 * Δt * (dθ[i] + dθ[i - 1])
            ∂dρ[i = 1:N], dρ[i] == dρ[i - 1] + 0.5 * Δt * (ddρ[i] + ddρ[i - 1])
            ∂dθ[i = 1:N], dθ[i] == dθ[i - 1] + 0.5 * Δt * (ddθ[i] + ddθ[i - 1])
            ∂dϕ[i = 1:N], dϕ[i] == dϕ[i - 1] + 0.5 * Δt * (ddϕ[i] + ddϕ[i - 1])
        end
    )

    # objective
    @objective(model, Min, tf)

    return model
end
