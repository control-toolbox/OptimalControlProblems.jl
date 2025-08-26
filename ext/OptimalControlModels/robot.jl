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

julia> docp, nlp = OptimalControlProblems.robot(OptimalControlBackend(); N=250);

julia> docp
DOCP object with 250 discretisation points

julia> nlp
NLP model corresponding to the robot arm problem
```
"""
function OptimalControlProblems.robot(::OptimalControlBackend; N::Int=250)

    # parameters

    # total length of arm
    L = 5

    # Upper bounds on the controls
    max_uρ = 1
    max_uθ = 1
    max_uϕ = 1

    # Initial positions of the length and the angles for the robot arm
    ρ0 = 4.5
    ϕ0 = π/4
    θf = 2π/3

    ocp = @def begin
        tf ∈ R, variable
        t ∈ [0, tf], time
        x = (ρ, dρ, θ, dθ, ϕ, dϕ) ∈ R⁶, state
        u = (uρ, uθ, uϕ) ∈ R³, control

        tf ≥ 0.1

        # state constraints
        0 ≤ ρ(t) ≤ L, (ρ_con)
        -π ≤ θ(t) ≤ π, (θ_con)
        0 ≤ ϕ(t) ≤ π, (ϕ_con)

        # control constraints
        -max_uρ ≤ uρ(t) ≤ max_uρ, (u_ρ_con)
        -max_uθ ≤ uθ(t) ≤ max_uθ, (u_θ_con)
        -max_uϕ ≤ uϕ(t) ≤ max_uϕ, (u_ϕ_con)

        # initial conditions
        ρ(0) == ρ0, (ρ0_con)
        ϕ(0) == ϕ0, (ϕ0_con)
        θ(0) == 0, (θ0_con)
        dθ(0) == 0, (dθ0_con)
        dϕ(0) == 0, (dϕ0_con)
        dρ(0) == 0, (dρ0_con)

        # final conditions
        ρ(tf) == ρ0, (ρf_con)
        θ(tf) == θf, (θf_con)
        ϕ(tf) == ϕ0, (ϕf_con)
        dθ(tf) == 0, (dθf_con)
        dϕ(tf) == 0, (dϕf_con)
        dρ(tf) == 0, (dρf_con)

        # aliases
        I_θ = ((L - ρ(t))^3 + ρ(t)^3) * sin(ϕ(t))^2
        I_ϕ = (L - ρ(t))^3 + ρ(t)^3

        # dynamics  
        ẋ(t) == [dρ(t), uρ(t) / L, dθ(t), 3 * uθ(t) / I_θ, dϕ(t), 3 * uϕ(t) / I_ϕ]

        # objective
        tf → min
    end

    # initial guess
    tf = 1
    xinit = t -> [ρ0, 0, 2π/3 * (t/tf)^2, 4π/3 * (t/tf), ϕ0, 0]
    uinit = [0, 0, 0]
    init = (state=xinit, control=uinit, variable=tf)

    # DOCP and NLP
    docp = direct_transcription(ocp; init=init, grid_size=N, disc_method=:trapeze)
    nlp = model(docp)

    return docp, nlp
end
