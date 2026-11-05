"""The **Jackson problem** is a classical benchmark in optimal control.

Builds the `jackson` problem with the JuMP backend.

```julia-repl
julia> using OptimalControlProblems
julia> jackson(JuMPBackend(); grid_size=100)
```

Default parameters and the modeling status are available through
`problem_metadata(:jackson)` and `OptimalControlProblems.status(:jackson)`.
"""
function OptimalControlProblems.jackson(
    ::JuMPBackend,
    args...;
    grid_size::Int=grid_size_data(:jackson),
    parameters::Union{Nothing,NamedTuple}=nothing,
    kwargs...,
)
    OptimalControlProblems._warn_if_draft(:jackson, :JuMP)

    # parameters
    params = parameters_data(:jackson, parameters)
    t0 = params[:t0]
    tf = params[:tf]
    k1 = params[:k1]
    k2 = params[:k2]
    k3 = params[:k3]
    a_l = params[:a_l]
    a_u = params[:a_u]
    b_l = params[:b_l]
    b_u = params[:b_u]
    x₃_l = params[:x₃_l]
    x₃_u = params[:x₃_u]
    u_l = params[:u_l]
    u_u = params[:u_u]
    a_t0 = params[:a_t0]
    b_t0 = params[:b_t0]
    x₃_t0 = params[:x₃_t0]

    function dynamics(x, u)
        a, b, _ = x
        return [-u[1] * (k1 * a - k2 * b), u[1] * (k1 * a - k2 * b) - (1 - u[1]) * k3 * b, (1 - u[1]) * k3 * b]
    end

    data = ProblemData(;
        state=[
            Component("a"; lower=a_l, upper=a_u, start=0.1),
            Component("b"; lower=b_l, upper=b_u, start=0.1),
            Component("x₃"; lower=x₃_l, upper=x₃_u, start=0.1),
        ],
        control=[Component("u"; lower=u_l, upper=u_u, start=0.1)],
        costate=["∂a", "∂b", "∂x₃"],
        times=v -> (t0, tf),
        dynamics=(x, u, v, t) -> dynamics(x, u),
        cost=(I, x0, xf, v) -> -xf[3],
        boundary=[
            (a_t0, (x0, xf, v) -> x0[1], a_t0),
            (b_t0, (x0, xf, v) -> x0[2], b_t0),
            (x₃_t0, (x0, xf, v) -> x0[3], x₃_t0),
        ],
    )
    return build_trapeze(data, grid_size, args...; kwargs...)
end
