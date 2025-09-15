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

julia> docp = OptimalControlProblems.cart_pendulum(OptimalControlBackend(); N=100);
```

# References

- Formulation inspired by OptimalControl approach for swing-up control problems.
"""
function OptimalControlProblems.cart_pendulum(
    ::OptimalControlBackend,
    description::Symbol...;
    grid_size::Int=grid_size_data(:cart_pendulum),
    parameters::Union{Nothing, NamedTuple}=nothing,
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
    max_tf = params[:max_tf]
    max_x = params[:max_x]
    max_v = params[:max_v]
    tf_l  = params[:tf_l]
    x_t0 = params[:x_t0]
    θ_t0 = params[:θ_t0]
    ω_t0 = params[:ω_t0]
    θ_tf = params[:θ_tf]
    ω_tf = params[:ω_tf]

    ocp = @def begin
        # time, variable, state and control
        w = (tf, ddx) ∈ R², variable
        t ∈ [t0, tf], time
        y = (x, v, θ, ω) ∈ R⁴, state
        Fex ∈ R, control

        # state constraints
        -max_x ≤ x(t) ≤ max_x, (x_c)
        -max_v ≤ v(t) ≤ max_v, (v_c)

        # control constraints
        -max_tf ≤ Fex(t) ≤ max_tf, (Fex_c)

        # variables constraints
        tf ≥ tf_l, (tf_c)

        # initial conditions
        x(t0) == x_t0, (x_t0)
        θ(t0) == θ_t0, (θ_t0)
        ω(t0) == ω_t0, (ω_t0)

        # final conditions
        θ(tf) == θ_tf, (θ_tf)
        ω(tf) == ω_tf, (ω_tf)

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
