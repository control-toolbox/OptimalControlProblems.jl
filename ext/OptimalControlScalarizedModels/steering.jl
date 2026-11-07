"""The **particle steering problem** is a classical benchmark in time-optimal control.

Builds the `steering` problem with the OptimalControlScalarized backend.

```julia-repl
julia> using OptimalControlProblems
julia> steering(OptimalControlScalarizedBackend())
```

Default parameters and the modeling status are available through
`problem_metadata(:steering)` and `OptimalControlProblems.status(:steering)`.
"""
function OptimalControlProblems.steering(
    ::OptimalControlScalarizedBackend; parameters::Union{Nothing,NamedTuple}=nothing
)
    OptimalControlProblems._warn_if_draft(:steering, :OptimalControlScalarized)

    # parameters
    params = parameters_data(:steering, parameters)
    t0 = params[:t0]
    a = params[:a]
    u_min = params[:u_min]
    u_max = params[:u_max]
    tf_l = params[:tf_l]
    x₁_t0 = params[:x₁_t0]
    x₂_t0 = params[:x₂_t0]
    x₃_t0 = params[:x₃_t0]
    x₄_t0 = params[:x₄_t0]
    x₂_tf = params[:x₂_tf]
    x₃_tf = params[:x₃_tf]
    x₄_tf = params[:x₄_tf]

    # Model
    ocp = @def begin
        tf ∈ R, variable
        t ∈ [t0, tf], time
        x ∈ R⁴, state
        u ∈ R, control

        x₁(t0) == x₁_t0, (x₁_t0)
        x₂(t0) == x₂_t0, (x₂_t0)
        x₃(t0) == x₃_t0, (x₃_t0)
        x₄(t0) == x₄_t0, (x₄_t0)
        x₂(tf) == x₂_tf, (x₂_tf)
        x₃(tf) == x₃_tf, (x₃_tf)
        x₄(tf) == x₄_tf, (x₄_tf)

        tf ≥ tf_l, (tf_c)
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
            return 5.0 * (t - t0)
        elseif i == 3
            return 45.0 * (t - t0)
        end
    end
    xinit = t -> [gen_x0(t, i) for i in 1:4]
    init = (state=xinit, control=0, variable=0.6)

    return OptimalControlProblems._problem(
        :steering, :OptimalControlScalarized, ocp, init, params
    )
end
