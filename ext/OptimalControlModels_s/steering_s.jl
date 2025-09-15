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
function OptimalControlProblems.steering_s(
    ::OptimalControlBackend,
    description::Symbol...;
    grid_size::Int=steps_number_data(:steering),
    parameters::Union{Nothing, NamedTuple}=nothing,
    kwargs...,
)

    # parameters
    params = parameters_data(:steering, parameters)
    t0 = params[:t0]
    a = params[:a]
    u_min = params[:u_min]
    u_max = params[:u_max]
    xs = params[:xs]
    yf = params[:yf]

    # Model
    ocp = @def begin
        tf ∈ R, variable
        t ∈ [t0, tf], time
        x ∈ R⁴, state
        u ∈ R, control

        tf ≥ 0, (tf_c)
        x(t0) == xs, (x_i)
        x[2:4](tf) == yf, (y_f)
        u_min ≤ u(t) ≤ u_max, (u_c)

        ∂(x₁)(t) == x₃(t)
        ∂(x₂)(t) == x₄(t)
        ∂(x₃)(t) == a * cos(u(t))
        ∂(x₄)(t) == a * sin(u(t))

        tf → min
    end

    # dynamics

    # initial guess
    function gen_x0(t, i)
        if i == 1 || i == 4
            return 0.0
        elseif i == 2
            return 5.0 * (t-t0)
        elseif i == 3
            return 45.0 * (t-t0)
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
        grid_size=grid_size,
        disc_method=:trapeze,
        kwargs...,
    )

    return docp
end
