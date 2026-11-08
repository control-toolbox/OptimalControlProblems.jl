"""The **ducted fan problem** is a classical nonlinear benchmark in optimal control with multiple input and state constraints.

Builds the `ducted_fan` problem with the JuMP backend.

```julia-repl
julia> using OptimalControlProblems
julia> ducted_fan(JuMPBackend(); grid_size=100)
```

The keyword `scheme` chooses the discretization: `:trapeze` (default), `:euler`,
`:euler_implicit` or `:midpoint`, as in `OptimalControl.Collocation`.
The keyword `ordering` chooses the order of the variables and constraints:
`:interleaved` (default, as in CTDirect) or `:blocks` (grouped by kind).

Default parameters and the modeling status are available through
`problem_metadata(:ducted_fan)` and `OptimalControlProblems.status(:ducted_fan)`.
"""
function OptimalControlProblems.ducted_fan(
    ::JuMPBackend,
    args...;
    grid_size::Int=grid_size_data(:ducted_fan),
    scheme::Symbol=:trapeze,
    ordering::Symbol=:interleaved,
    parameters::Union{Nothing,NamedTuple}=nothing,
    init=nothing,
    kwargs...,
)
    OptimalControlProblems.Catalogue._warn_if_draft(:ducted_fan, :JuMP)

    # parameters
    params = parameters_data(:ducted_fan, parameters)
    t0 = params[:t0]
    r = params[:r]
    J = params[:J]
    m = params[:m]
    mg = params[:mg]
    μ = params[:μ]
    α_l = params[:α_l]
    α_u = params[:α_u]
    u₁_l = params[:u₁_l]
    u₁_u = params[:u₁_u]
    u₂_l = params[:u₂_l]
    u₂_u = params[:u₂_u]
    tf_l = params[:tf_l]
    x₁_t0 = params[:x₁_t0]
    v₁_t0 = params[:v₁_t0]
    x₂_t0 = params[:x₂_t0]
    v₂_t0 = params[:v₂_t0]
    α_t0 = params[:α_t0]
    vα_t0 = params[:vα_t0]
    x₁_tf = params[:x₁_tf]
    v₁_tf = params[:v₁_tf]
    x₂_tf = params[:x₂_tf]
    v₂_tf = params[:v₂_tf]
    α_tf = params[:α_tf]
    vα_tf = params[:vα_tf]

    function dynamics(x, u)
        _, v₁, _, v₂, α, vα = x
        u₁, u₂ = u
        return [
            v₁,
            (u₁ * cos(α) - u₂ * sin(α)) / m,
            v₂,
            (-mg + u₁ * sin(α) + u₂ * cos(α)) / m,
            vα,
            r * u₁ / J,
        ]
    end

    data = ProblemData(;
        state=[
            Component("x₁"),
            Component("v₁"),
            Component("x₂"),
            Component("v₂"),
            Component("α"; lower=α_l, upper=α_u),  # radian
            Component("vα"),
        ],
        control=[
            Component("u₁"; lower=u₁_l, upper=u₁_u),  # [N]
            Component("u₂"; lower=u₂_l, upper=u₂_u),  # [N]
        ],
        variable=[Component("tf"; lower=tf_l)],
        costate=["∂x₁", "∂v₁", "∂x₂", "∂v₂", "∂α", "∂vα"],
        times=v -> (t0, v[1]),
        dynamics=(x, u, v, t) -> dynamics(x, u),
        lagrange=(x, u, v, t) -> 2 * u[1]^2 + u[2]^2,
        cost=(I, x0, xf, v) -> (1 / v[1]) * I + (μ * v[1]),
        boundary=[
            (x₁_t0, (x0, xf, v) -> x0[1], x₁_t0),
            (v₁_t0, (x0, xf, v) -> x0[2], v₁_t0),
            (x₂_t0, (x0, xf, v) -> x0[3], x₂_t0),
            (v₂_t0, (x0, xf, v) -> x0[4], v₂_t0),
            (α_t0, (x0, xf, v) -> x0[5], α_t0),
            (vα_t0, (x0, xf, v) -> x0[6], vα_t0),
            (x₁_tf, (x0, xf, v) -> xf[1], x₁_tf),
            (v₁_tf, (x0, xf, v) -> xf[2], v₁_tf),
            (x₂_tf, (x0, xf, v) -> xf[3], x₂_tf),
            (v₂_tf, (x0, xf, v) -> xf[4], v₂_tf),
            (α_tf, (x0, xf, v) -> xf[5], α_tf),
            (vα_tf, (x0, xf, v) -> xf[6], vα_tf),
        ],
    )
    # initial guess: from the metadata (#377), unless given
    init = OptimalControlProblems.Metadata._initial_guess(:ducted_fan, params, init)
    return build(data, grid_size, scheme, ordering, args...; init=init, kwargs...)
end
