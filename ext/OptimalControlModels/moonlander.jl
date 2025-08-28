"""
$(TYPEDSIGNATURES)

Constructs an **OptimalControl problem** for the Moonlander benchmark model.  
The function defines the state and control variables, system dynamics, bounds, initial and final conditions, and the cost functional, which minimises the landing time of a moonlander to a specified target.  
It returns both a discretised direct optimal control problem (DOCP) and the corresponding nonlinear programming (NLP) model.

# Arguments

- `::OptimalControlBackend`: Placeholder type specifying the OptimalControl backend or solver interface.
- `N::Int=500`: (Keyword) Number of discretisation points for the direct transcription grid.

# Returns

- `docp`: The direct optimal control problem object representing the Moonlander problem.
- `nlp`: The corresponding nonlinear programming model obtained from the DOCP, suitable for numerical optimisation.

# Example

```julia-repl
julia> using OptimalControlProblems

julia> docp = OptimalControlProblems.moonlander(OptimalControlBackend(); N=500);
```
"""
function OptimalControlProblems.moonlander(
    ::OptimalControlBackend, description::Symbol...; N::Int=steps_number_data(:moonlander), kwargs...
)

    # parameters
    target=[5.0, 5.0]
    m = 1
    g = 9.81
    I = 0.1
    D = 1
    max_thrust = 2g

    # define the problem
    ocp = @def begin

        # state, control and final time variables, and time
        tf ∈ R, variable
        t ∈ [0, tf], time
        x = (p1, p2, dp1, dp2, θ, dθ) ∈ R⁶, state
        u = (F1, F2) ∈ R², control

        # final time constraint
        0.1 ≤ tf ≤ 1.0

        # control constraints
        0 ≤ F1(t) ≤ max_thrust, (F1_con)
        0 ≤ F2(t) ≤ max_thrust, (F2_con)

        # initial conditions
        p1(0) == 0, (p1_ic)
        p2(0) == 0, (p2_ic)
        dp1(0) == 0, (dp1_ic)
        dp2(0) == 0, (dp2_ic)
        θ(0) == 0, (θ_ic)
        dθ(0) == 0, (dθ_ic)

        # final conditions
        p1(tf) == target[1], (p1_fc)
        p2(tf) == target[2], (p2_fc)
        dp1(tf) == 0, (dp1_fc)
        dp2(tf) == 0, (dp2_fc)

        # dynamics
        ẋ(t) == dynamics(x(t), u(t))

        # objective
        tf → min
    end

    # dynamics
    function dynamics(x, u)
        p1, p2, dp1, dp2, θ, dθ = x
        F1, F2 = u

        F_r = [
            cos(θ) -sin(θ) p1
            sin(θ) cos(θ) p2
            0 0 1
        ]
        F_tot = (F_r * [0; F1 + F2; 0])[1:2]
        ddp1 = (1 / m) * F_tot[1]
        ddp2 = (1 / m) * F_tot[2] - g
        ddθ = (1 / I) * (D / 2) * (F2 - F1)

        return [dp1, dp2, ddp1, ddp2, dθ, ddθ]
    end

    # Initial guess
    xinit = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]  # [p1, p2, dp1, dp2, θ, dθ]
    uinit = [5.0, 5.0]  # [F1, F2] 
    varinit = [0.5]  # [tf] 
    init = (state=xinit, control=uinit, variable=varinit)

    # DOCP and NLP
    docp = direct_transcription(ocp, description...; init=init, grid_size=N, disc_method=:trapeze, kwargs...)

    return docp
end
