"""
$(TYPEDSIGNATURES)

Constructs an **OptimalControl problem** for a robotic arm moving between two points.  
This function defines the state and control variables, system dynamics, initial and final conditions, and the cost functional, which minimises the total time taken to perform the motion.  
Reference: Robot arm problem on BOCOP [here](https://github.com/control-toolbox/bocop/tree/main/bocop)

# Arguments

- `::OptimalControlBackend`: Placeholder type specifying the OptimalControl backend or solver interface.
- `N::Int=250`: (Keyword) Number of discretisation points for the direct transcription grid.

# Returns

- `docp`: The direct optimal control problem object representing the robot arm problem.
- `nlp`: The corresponding nonlinear programming model obtained from the DOCP, suitable for numerical optimisation.

# Example

```julia-repl
julia> using OptimalControlProblems

julia> docp = OptimalControlProblems.robot(OptimalControlBackend(); N=250);
```
"""
function OptimalControlProblems.robot_s(
    ::OptimalControlBackend,
    description::Symbol...;
    grid_size::Int=steps_number_data(:robot),
    parameters::Union{Nothing, NamedTuple}=nothing,
    kwargs...,
)

    # parameters
    params = parameters_data(:robot, parameters)
    t0 = params[:t0]

    # total length of arm
    L = params[:L]

    # Upper bounds on the controls
    max_uρ = params[:max_uρ]
    max_uθ = params[:max_uθ]
    max_uϕ = params[:max_uϕ]

    # Initial positions of the length and the angles for the robot arm
    ρ0 = params[:ρ0]
    ϕ0 = params[:ϕ0]
    θf = params[:θf]

    ocp = @def begin
        tf ∈ R, variable
        t ∈ [t0, tf], time
        x = (ρ, dρ, θ, dθ, ϕ, dϕ) ∈ R⁶, state
        u = (uρ, uθ, uϕ) ∈ R³, control

        tf ≥ 0.1

        # state constraints
        0 ≤ ρ(t) ≤ L, (ρ_c)
        -π ≤ θ(t) ≤ π, (θ_c)
        0 ≤ ϕ(t) ≤ π, (ϕ_c)

        # control constraints
        -max_uρ ≤ uρ(t) ≤ max_uρ, (u_ρ_c)
        -max_uθ ≤ uθ(t) ≤ max_uθ, (u_θ_c)
        -max_uϕ ≤ uϕ(t) ≤ max_uϕ, (u_ϕ_c)

        # initial conditions
        ρ(t0) == ρ0, (ρ_i)
        ϕ(t0) == ϕ0, (ϕ_i)
        θ(t0) == 0, (θ_i)
        dθ(t0) == 0, (dθ_i)
        dϕ(t0) == 0, (dϕ_i)
        dρ(t0) == 0, (dρ0_i)

        # final conditions
        ρ(tf) == ρ0, (ρ_f)
        θ(tf) == θf, (θ_f)
        ϕ(tf) == ϕ0, (ϕ_f)
        dθ(tf) == 0, (dθ_f)
        dϕ(tf) == 0, (dϕ_f)
        dρ(tf) == 0, (dρ_f)

        # aliases
        I_θ = ((L - ρ(t))^3 + ρ(t)^3) * sin(ϕ(t))^2
        I_ϕ = (L - ρ(t))^3 + ρ(t)^3

        # dynamics  
        ∂(ρ)(t) == dρ(t)
        ∂(dρ)(t) == uρ(t) / L
        ∂(θ)(t) == dθ(t)
        ∂(dθ)(t) == 3 * uθ(t) / I_θ
        ∂(ϕ)(t) == dϕ(t)
        ∂(dϕ)(t) == 3 * uϕ(t) / I_ϕ

        # objective
        tf → min
    end

    # initial guess
    tf = 1
    xinit = t -> [ρ0, 0, 2π/3 * ((t - t0) / (tf - t0))^2, 4π/3 * ((t - t0) / (tf - t0)), ϕ0, 0]
    uinit = [0, 0, 0]
    init = (state=xinit, control=uinit, variable=tf)

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
