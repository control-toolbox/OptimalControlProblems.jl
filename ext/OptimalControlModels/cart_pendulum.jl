"""
$(TYPEDSIGNATURES)

Constructs an **OptimalControl problem** representing the Cart-Pendulum system.  
The function defines state and control variables, boundary conditions, path constraints, and system dynamics, with the objective of swinging the pendulum from the downward position to the upright position in minimum time.  
It performs direct transcription to produce a discretised optimal control problem (DOCP) and the corresponding nonlinear programming (NLP) model.

# Arguments

- `::OptimalControlBackend`: Placeholder type specifying the OptimalControl backend or solver interface.
- `N::Int=500`: (Keyword) Number of discretisation points for the direct transcription grid.

# Returns

- `docp`: The direct optimal control problem object representing the discretised Cart-Pendulum problem.
- `nlp`: The corresponding nonlinear programming model obtained from the DOCP, suitable for numerical optimisation.

# Example

```julia-repl
julia> using OptimalControlProblems

julia> docp, nlp = OptimalControlProblems.cart_pendulum(OptimalControlBackend(); N=100);

julia> docp
DOCP object with 100 discretisation points

julia> nlp
NLP model corresponding to the Cart-Pendulum problem
```

# References

- Formulation inspired by OptimalControl approach for swing-up control problems.
"""
function OptimalControlProblems.cart_pendulum(::OptimalControlBackend; N::Int=steps_number_data(:cart_pendulum))

    # parameters
    g = 9.81            # gravitation [m/s^2]
    L = 1               # pendulum length [m]
    m = 1               # pendulum mass [kg]
    I = m * L^2 / 12    # pendulum moment of inertia
    mcart = 0.5        # cart mass [kg]
    max_f = 5
    max_x = 1
    max_v = 2

    ocp = @def begin

        # time, variable, state and control
        w = (tf, ddx) ∈ R², variable
        t ∈ [0, tf], time
        y = (x, v, θ, ω) ∈ R⁴, state
        Fex ∈ R, control

        # state constraints
        -max_x ≤ x(t) ≤ max_x, (x_con)
        -max_v ≤ v(t) ≤ max_v, (v_con)

        # control constraints
        -max_f ≤ Fex(t) ≤ max_f, (Fex_con)

        # variables constraints
        tf ≥ 0.1, (tf_con)

        # initial conditions
        x(0) == 0, (x_ic)
        θ(0) == 0, (θ_ic)
        ω(0) == 0, (ω_ic)

        # final conditions
        θ(tf) == π, (θ_fc)
        ω(tf) == 0, (ω_fc)

        # dynamics
        ẏ(t) == dynamics(v(t), θ(t), ω(t), Fex(t), ddx)

        # objective
        tf → min
    end

    # dynamics
    function dynamics(v, θ, ω, Fex, ddx)

        #
        α(ddx) = 1 / (I + 0.25 * m * L^2) * 0.5 * L * m * (-ddx * cos(θ) - g * sin(θ))
        ddCOG = L * ω * [-sin(θ), cos(θ)] + L / 2 * [cos(θ), sin(θ)] * α(ddx) + [ddx, 0]
        FXFY = m * ddCOG + [0, m * g]
        eq = -FXFY[1] + Fex - mcart * ddx # # eq = J ddx + c
        J = mcart # should be -(m+mcart) but was mcart?
        c = eq - J * ddx

        #
        ẋ = v
        v̇ = -1 / J * c
        θ̇ = ω
        ω̇ = α(v̇)

        return [ẋ, v̇, θ̇, ω̇]
    end

    # initial guess
    xinit = [0.1, 0.1, 0.1, 0.1]    # [x, v, θ, ω]
    uinit = [0.1]                   # [Fex]
    varinit = [1.0, 0.1]            # [tf, ddx]
    init = (state=xinit, control=uinit, variable=varinit)

    # NLPModel + DOCP
    docp = direct_transcription(ocp; init=init, grid_size=N, disc_method=:trapeze)
    nlp = model(docp)
    return docp, nlp
end
