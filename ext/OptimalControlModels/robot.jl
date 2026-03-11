"""
$(TYPEDSIGNATURES)

Constructs an **OptimalControl problem** for a robotic arm moving between two points.  
This function defines the state and control variables, system dynamics, initial and final conditions, and the cost functional, which minimises the total time taken to perform the motion.  
Reference: Robot arm problem on BOCOP [here](https://github.com/control-toolbox/bocop/tree/main/bocop)

# Arguments

- `::OptimalControlBackend`: Placeholder type specifying the OptimalControl backend or solver interface.
- `grid_size::Int=250`: (Keyword) Number of discretisation points for the direct transcription grid.

# Returns

- `docp`: The direct optimal control problem object representing the robot arm problem.
- `nlp`: The corresponding nonlinear programming model obtained from the DOCP, suitable for numerical optimisation.

# Example

```julia-repl
julia> using OptimalControlProblems

julia> docp = OptimalControlProblems.robot(OptimalControlBackend(); N=250);
```
"""
function OptimalControlProblems.robot(
    ::OptimalControlBackend,
    description::Symbol...;
    grid_size::Int=grid_size_data(:robot),
    parameters::Union{Nothing,NamedTuple}=nothing,
    kwargs...,
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

    ocp = @def begin
        tf ∈ R, variable
        t ∈ [t0, tf], time
        x = (ρ, dρ, θ, dθ, ϕ, dϕ) ∈ R⁶, state
        u = (uρ, uθ, uϕ) ∈ R³, control

        tf ≥ tf_l

        # state constraints
        ρ_l ≤ ρ(t) ≤ ρ_u, (ρ_c)
        θ_l ≤ θ(t) ≤ θ_u, (θ_c)
        ϕ_l ≤ ϕ(t) ≤ ϕ_u, (ϕ_c)

        # control constraints
        uρ_l ≤ uρ(t) ≤ uρ_u, (u_ρ_c)
        uθ_l ≤ uθ(t) ≤ uθ_u, (u_θ_c)
        uϕ_l ≤ uϕ(t) ≤ uϕ_u, (u_ϕ_c)

        # initial conditions
        ρ(t0) == ρ_t0, (ρ_t0)
        θ(t0) == θ_t0, (θ_t0)
        ϕ(t0) == ϕ_t0, (ϕ_t0)
        dρ(t0) == dρ_t0, (dρ_t0)
        dθ(t0) == dθ_t0, (dθ_t0)
        dϕ(t0) == dϕ_t0, (dϕ_t0)

        # final conditions
        ρ(tf) == ρ_tf, (ρ_tf)
        θ(tf) == θ_tf, (θ_tf)
        ϕ(tf) == ϕ_tf, (ϕ_tf)
        dρ(tf) == dρ_tf, (dρ_tf)
        dθ(tf) == dθ_tf, (dθ_tf)
        dϕ(tf) == dϕ_tf, (dϕ_tf)

        I_θ = ((L - ρ(t))^3 + ρ(t)^3) * (sin(ϕ(t))^2 + 1e-9)
        I_ϕ = (L - ρ(t))^3 + ρ(t)^3

        # dynamics  
        ẋ(t) == [dρ(t), uρ(t) / L, dθ(t), 3 * uθ(t) / I_θ, dϕ(t), 3 * uϕ(t) / I_ϕ]

        # objective
        tf → min
    end

    # initial guess
    tf_guess = 9.1
    xinit = t -> begin
        alpha = clamp((t - t0) / (tf_guess - t0), 0.0, 1.0)

        ρ_val = ρ_t0 + alpha * (ρ_tf - ρ_t0)
        θ_val = θ_t0 + alpha * (θ_tf - θ_t0)
        ϕ_val = ϕ_t0 + alpha * (ϕ_tf - ϕ_t0)

        if abs(ϕ_val) < 1e-6
            ϕ_val = 1e-6
        end

        return [ρ_val, 0.0, θ_val, 0.0, ϕ_val, 0.0]
    end

    uinit = [0.0, 0.0, 0.0]
    init = (state=xinit, control=uinit, variable=tf_guess)

    # discretise the optimal control problem
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
