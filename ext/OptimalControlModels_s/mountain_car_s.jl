"""
$(TYPEDSIGNATURES)

Constructs an **OptimalControl problem** for the Mountain Car problem (symbolic version).  
The goal is to drive an underpowered car up a steep hill.  
The objective is to minimize the time required to reach the target position.  
The problem formulation can be found [here](https://openmdao.github.io/dymos/examples/mountain_car/mountain_car.html).

# Arguments

- `::OptimalControlBackend`: Placeholder type specifying the OptimalControl backend or solver interface.
- `grid_size::Int=100`: (Keyword) Number of discretisation points for the direct transcription grid.

# Returns

- `docp`: The direct optimal control problem object representing the Mountain Car problem.

# Example

```julia-repl
julia> using OptimalControlProblems

julia> docp = OptimalControlProblems.mountain_car_s(OptimalControlBackend(); N=100);
```
"""
function OptimalControlProblems.mountain_car_s(
    ::OptimalControlBackend,
    description::Symbol...;
    grid_size::Int=grid_size_data(:mountain_car),
    parameters::Union{Nothing,NamedTuple}=nothing,
    kwargs...,
)

    # parameters
    params = parameters_data(:mountain_car, parameters)
    t0 = params[:t0]
    tf_start = params[:tf_start]
    tf_min = params[:tf_min]
    pos_t0 = params[:pos_t0]
    vel_t0 = params[:vel_t0]
    pos_tf = params[:pos_tf]
    vel_tf_min = params[:vel_tf_min]
    pos_min = params[:pos_min]
    pos_max = params[:pos_max]
    vel_min = params[:vel_min]
    vel_max = params[:vel_max]
    u_min = params[:u_min]
    u_max = params[:u_max]
    a = params[:a]
    b = params[:b]
    c = params[:c]

    # model
    ocp = @def begin
        tf ∈ R, variable
        t ∈ [t0, tf], time
        x = (pos, vel) ∈ R², state
        u ∈ R, control

        # constraints
        tf ≥ tf_min
        pos_min ≤ pos(t) ≤ pos_max
        vel_min ≤ vel(t) ≤ vel_max
        u_min ≤ u(t) ≤ u_max

        # initial conditions
        pos(t0) == pos_t0
        vel(t0) == vel_t0

        # final conditions
        pos(tf) == pos_tf
        vel(tf) ≥ vel_tf_min

        # dynamics
        ∂(pos)(t) == vel(t)
        ∂(vel)(t) == a * u(t) - b * cos(c * pos(t))

        # objective
        tf → min
    end

    # initial guess
    N = grid_size
    xinit = [[pos_t0 + (pos_tf - pos_t0) * i / N, 0.0] for i in 0:N]
    uinit = 0.0
    time_vec = LinRange(t0, tf_start, N + 1)
    init = (time=time_vec, state=xinit, control=uinit, variable=tf_start)

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
