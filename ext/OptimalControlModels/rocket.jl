"""
$(TYPEDSIGNATURES)

Constructs an **OptimalControl problem** for the Goddard rocket.  
This function defines the state variables (altitude, velocity, mass), the control variable (thrust), system dynamics, constraints, initial and final conditions, and the cost functional, which maximises the final altitude.  
Reference: Goddard Rocket Problem [here](https://github.com/control-toolbox/bocop/tree/main/bocop)

# Arguments

- `::OptimalControlBackend`: Placeholder type specifying the OptimalControl backend or solver interface.
- `N::Int=500`: (Keyword) Number of discretisation points for the direct transcription grid.

# Returns

- `docp`: The direct optimal control problem object representing the Goddard rocket problem.
- `nlp`: The corresponding nonlinear programming model obtained from the DOCP, suitable for numerical optimisation.

# Example

```julia-repl
julia> using OptimalControlProblems

julia> docp = OptimalControlProblems.rocket(OptimalControlBackend(); N=500);
```
"""
function OptimalControlProblems.rocket(
    ::OptimalControlBackend,
    description::Symbol...;
    grid_size::Int=steps_number_data(:rocket),
    parameters::Union{Nothing, NamedTuple}=nothing,
    kwargs...,
)

    # parameters
    params = parameters_data(:rocket, parameters)
    t0 = params[:t0]
    h0 = params[:h0]
    v0 = params[:v0]
    m0 = params[:m0]
    g0 = params[:g0]
    Tc = params[:Tc]
    hc = params[:hc]
    vc = params[:vc]
    mc = params[:mc]

    #
    c = 0.5 * sqrt(g0 * h0)
    mf = mc * m0
    Dc = 0.5 * vc * (m0 / g0)
    Tmax = Tc * m0 * g0

    # Model
    ocp = @def begin
        tf ∈ R, variable
        t ∈ [t0, tf], time
        x = (h, v, m) ∈ R³, state
        T ∈ R, control

        # state constraints
        h(t) ≥ h0, (x1_c)
        v(t) ≥ v0, (x2_c)
        mf ≤ m(t) ≤ m0, (x3_c)

        # control constraints
        0 ≤ T(t) ≤ Tmax, (T_c)

        # time constraints
        tf ≥ 0, (tf_c)

        # initial conditions
        h(t0) == h0, (x1_i)
        v(t0) == v0, (x2_i)
        m(t0) == m0, (x3_i)

        # final conditions
        m(tf) == mf, (x3_f)

        # dynamics
        ẋ(t) == dynamics(h(t), v(t), m(t), T(t))

        # objective
        -h(tf) → min
    end

    # dynamics
    function dynamics(h, v, m, T)
        D = (Dc * v^2 * exp(-hc * (h - h0)) / h0)
        g = g0 * (h0 / h)^2
        return [v, (T - D - m * g) / m, -T / c]
    end

    # initial guess
    tf_init = 1
    xinit = [[1, i / N * (1 - i / N), (mf - m0) * (i / N) + m0] for i in 0:N]
    time_vec = LinRange(0, tf_init, N+1)
    init = (time=time_vec, state=xinit, control=Tmax/2, variable=tf_init)

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
