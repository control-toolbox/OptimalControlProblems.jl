"""
$(TYPEDSIGNATURES)

Constructs and returns a JuMP model for the **Moonlander Optimal Control Problem**.  
The model represents the dynamics of a moonlander with two translational and one rotational degrees of freedom.  
The objective is to minimise the final landing time (`tf`) while ensuring the lander reaches the target position with zero velocity.  
The dynamics include translational acceleration, rotation, and thrust allocation, discretised with `N` steps using trapezoidal collocation.

# Arguments

- `::JuMPBackend`: Specifies the backend for building the JuMP model.
- `N::Int=500`: (Keyword) Number of discretisation steps in the time grid.

# Returns

- `model::JuMP.Model`: A JuMP model representing the Moonlander optimal control problem.

# Example

```julia-repl
julia> using OptimalControlProblems
julia> using JuMP

julia> model = OptimalControlProblems.moonlander(JuMPBackend(); N=100)
```

# References

- Problem formulation available at: https://arxiv.org/pdf/2303.16746
"""
function OptimalControlProblems.moonlander(
    ::JuMPBackend, args...; grid_size::Int=grid_size_data(:moonlander), 
    parameters::Union{Nothing, NamedTuple}=nothing,
    kwargs...
)

    # parameters
    params = parameters_data(:moonlander, parameters)
    t0 = params[:t0]
    m = params[:m]
    g = params[:g]
    I = params[:I]
    D = params[:D]
    max_thrust = params[:max_thrust]
    tf_l = params[:tf_l]
    tf_u = params[:tf_u]
    F₁_l = params[:F₁_l]
    F₁_u = max_thrust
    F₂_l = params[:F₂_l]
    F₂_u = max_thrust
    tf_l = params[:tf_l]
    tf_u = params[:tf_u]
    F₁_l = params[:F₁_l]
    F₂_l = params[:F₂_l]
    p₁_t0 = params[:p₁_t0]
    p₂_t0 = params[:p₂_t0]
    dp₁_t0 = params[:dp₁_t0]
    dp₂_t0 = params[:dp₂_t0]
    θ_t0 = params[:θ_t0]
    dθ_t0 = params[:dθ_t0]
    p₁_tf = params[:p₁_tf]
    p₂_tf = params[:p₂_tf]
    dp₁_tf = params[:dp₁_tf]
    dp₂_tf = params[:dp₂_tf]
        
    # define the problem
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

    # state, control and final time variables
    @variables(
        model,
        begin
            # final time
            tf_l ≤ tf ≤ tf_u, (start = 0.5)

            # state variables
            p₁[k = 0:N], (start = 0.1)
            p₂[k = 0:N], (start = 0.1)
            dp₁[k = 0:N], (start = 0.1)
            dp₂[k = 0:N], (start = 0.1)
            θ[k = 0:N], (start = 0.1)
            dθ[k = 0:N], (start = 0.1)

            # control variables
            F₁_l ≤ F₁[k = 0:N] ≤ F₁_u, (start = 5.0)
            F₂_l ≤ F₂[k = 0:N] ≤ F₂_u, (start = 5.0)
        end
    )

    # initial and final conditions
    @constraints(
        model,
        begin
            p₁[0] == p₁_t0
            p₂[0] == p₂_t0
            dp₁[0] == dp₁_t0
            dp₂[0] == dp₂_t0
            θ[0] == θ_t0
            dθ[0] == dθ_t0

            p₁[N] == p₁_tf
            p₂[N] == p₂_tf
            dp₁[N] == dp₁_tf
            dp₂[N] == dp₂_tf
        end
    )

    # dynamics
    @expressions(
        model,
        begin
            F_r[k = 0:N], [
                cos(θ[k]) -sin(θ[k]) p₁[k]
                sin(θ[k]) cos(θ[k]) p₂[k]
                0 0 1
            ]
        end
    )
    @expressions(
        model,
        begin
            F_tot[k = 0:N], (F_r[k] * [0; F₁[k] + F₂[k]; 0])[1:2]
        end
    )
    @expressions(
        model,
        begin
            #
            Δt, (tf - t0) / N

            #
            ddp₁[k = 0:N], (1 / m) * F_tot[k][1]
            ddp₂[k = 0:N], (1 / m) * F_tot[k][2] - g
            ddθ[k = 0:N], (1 / I) * (D / 2) * (F₂[k] - F₁[k])
        end
    )

    @constraints(
        model,
        begin
            ∂p₁[k = 1:N], p₁[k] == p₁[k - 1] + 0.5 * Δt * (dp₁[k] + dp₁[k - 1])
            ∂p₂[k = 1:N], p₂[k] == p₂[k - 1] + 0.5 * Δt * (dp₂[k] + dp₂[k - 1])
            ∂dp₁[k = 1:N], dp₁[k] == dp₁[k - 1] + 0.5 * Δt * (ddp₁[k] + ddp₁[k - 1])
            ∂dp₂[k = 1:N], dp₂[k] == dp₂[k - 1] + 0.5 * Δt * (ddp₂[k] + ddp₂[k - 1])
            ∂θ[k = 1:N], θ[k] == θ[k - 1] + 0.5 * Δt * (dθ[k] + dθ[k - 1])
            ∂dθ[k = 1:N], dθ[k] == dθ[k - 1] + 0.5 * Δt * (ddθ[k] + ddθ[k - 1])
        end
    )

    @objective(model, Min, tf)

    return model
end
