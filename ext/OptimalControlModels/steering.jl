"""
$(TYPEDSIGNATURES)

Constructs an **OptimalControl problem** for steering a particle along a trajectory.  
The objective is to minimise the total time taken to reach a specified terminal state, subject to control limits and particle dynamics.  
The state vector has four components, and the control is a single scalar input.  

# Arguments

- `::OptimalControlBackend`: Placeholder type specifying the OptimalControl backend or solver interface.
- `N::Int=500`: (Keyword) Number of discretisation points for the direct transcription grid.

# Returns

- `docp`: The direct optimal control problem object representing the particle steering problem.
- `nlp`: The corresponding nonlinear programming model obtained from the DOCP, suitable for numerical optimisation.

# Example

```julia-repl
julia> using OptimalControlProblems

julia> docp = OptimalControlProblems.steering(OptimalControlBackend(); N=500);
```
"""
function OptimalControlProblems.steering(
    ::OptimalControlBackend,
    description::Symbol...;
    N::Int=steps_number_data(:steering),
    kwargs...,
)

    # parameters
    a = 100
    u_min = -π/2
    u_max = π/2
    xs = zeros(4)
    xf = [NaN, 5, 45, 0]

    # Model
    ocp = @def begin
        tf ∈ R, variable
        t ∈ [0.0, tf], time
        x ∈ R⁴, state
        u ∈ R¹, control

        tf ≥ 0, (tf_con)
        x(0) == xs, (x_ic)
        x(tf) == xf, (x_fc)
        u_min ≤ u(t) ≤ u_max, (u_con)

        ẋ(t) == dynamics(x(t), u(t))

        tf → min
    end

    # dynamics
    function dynamics(x, u)
        return [x[3], x[4], a * cos(u), a * sin(u)]
    end

    # initial guess
    function gen_x0(t, i)
        if i == 1 || i == 4
            return 0.0
        elseif i == 2
            return 5.0 * t
        elseif i == 3
            return 45.0 * t
        end
    end
    xinit = t -> [gen_x0(t, i) for i in 1:4]
    init = (state=xinit, control=0, variable=1)

    # discretise the optimal control problem
    docp = direct_transcription(
        ocp,
        description...;
        lagrange_to_mayer=false,
        init=init,
        grid_size=N,
        disc_method=:trapeze,
        kwargs...,
    )

    return docp
end
