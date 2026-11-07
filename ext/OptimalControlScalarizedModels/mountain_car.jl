"""The **Mountain Car problem** is a classic benchmark in control and reinforcement learning.

Builds the `mountain_car` problem with the OptimalControlScalarized backend.

```julia-repl
julia> using OptimalControlProblems
julia> mountain_car(OptimalControlScalarizedBackend())
```

Default parameters and the modeling status are available through
`problem_metadata(:mountain_car)` and `OptimalControlProblems.status(:mountain_car)`.
"""
function OptimalControlProblems.mountain_car(
    ::OptimalControlScalarizedBackend; parameters::Union{Nothing,NamedTuple}=nothing
)
    OptimalControlProblems._warn_if_draft(:mountain_car, :OptimalControlScalarized)

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
    init = (state=[pos_t0, vel_t0], control=0.0, variable=tf_start)

    return OptimalControlProblems._problem(
        :mountain_car, :OptimalControlScalarized, ocp, init, params
    )
end
