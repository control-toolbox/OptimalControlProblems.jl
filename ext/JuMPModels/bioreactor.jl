"""The **photobioreactor–digester system problem** is a benchmark in constrained optimal control.

Builds the `bioreactor` problem with the JuMP backend.

```julia-repl
julia> using OptimalControlProblems
julia> bioreactor(JuMPBackend(); grid_size=100)
```

The keyword `scheme` chooses the discretization: `:trapeze` (default), `:euler`,
`:euler_implicit` or `:midpoint`, as in `OptimalControl.Collocation`.
The keyword `ordering` chooses the order of the variables and constraints:
`:interleaved` (default, as in CTDirect) or `:blocks` (grouped by kind).

Default parameters and the modeling status are available through
`problem_metadata(:bioreactor)` and `OptimalControlProblems.status(:bioreactor)`.
"""
function OptimalControlProblems.bioreactor(
    ::JuMPBackend,
    args...;
    grid_size::Int=grid_size_data(:bioreactor),
    scheme::Symbol=:trapeze,
    ordering::Symbol=:interleaved,
    parameters::Union{Nothing,NamedTuple}=nothing,
    init=nothing,
    kwargs...,
)
    OptimalControlProblems.Catalogue._warn_if_draft(:bioreactor, :JuMP)

    # parameters
    params = parameters_data(:bioreactor, parameters)
    t0 = params[:t0]
    tf = params[:tf]
    β = params[:β]
    c = params[:c]
    γ = params[:γ]
    halfperiod = params[:halfperiod]
    Ks = params[:Ks]
    μ2m = params[:μ2m]
    μbar = params[:μbar]
    r = params[:r]
    y_l = params[:y_l]
    s_l = params[:s_l]
    b_l = params[:b_l]
    u_l = params[:u_l]
    u_u = params[:u_u]
    y_t0_l = params[:y_t0_l]
    y_t0_u = params[:y_t0_u]
    s_t0_l = params[:s_t0_l]
    s_t0_u = params[:s_t0_u]
    b_t0_l = params[:b_t0_l]
    b_t0_u = params[:b_t0_u]

    function dynamics(x, u, t)
        y, s, b = x
        μ2 = μ2m * s / (s + Ks)
        days = (t - t0) / (halfperiod * 2)
        tau = (days - floor(days)) * 2π
        μ = max(0, sin(tau))^2 * μbar
        return [
            μ * y / (1 + y) - (r + u[1]) * y,
            -μ2 * b + u[1] * β * (γ * y - s),
            (μ2 - u[1] * β) * b,
        ]
    end

    data = ProblemData(;
        state=[
            Component("y"; lower=y_l),
            Component("s"; lower=s_l),
            Component("b"; lower=b_l),
        ],
        control=[Component("u"; lower=u_l, upper=u_u)],
        costate=["∂y", "∂s", "∂b"],
        times=v -> (t0, tf),
        dynamics=(x, u, v, t) -> dynamics(x, u, t),
        lagrange=(x, u, v, t) -> μ2m * x[2] / (x[2] + Ks) * x[3] / (β + c),
        cost=(I, x0, xf, v) -> -I,
        boundary=[
            (y_t0_l, (x0, xf, v) -> x0[1], y_t0_u),
            (s_t0_l, (x0, xf, v) -> x0[2], s_t0_u),
            (b_t0_l, (x0, xf, v) -> x0[3], b_t0_u),
        ],
    )
    # initial guess: from the metadata (#377), unless given
    init = OptimalControlProblems.Metadata._initial_guess(:bioreactor, params, init)
    return build(data, grid_size, scheme, ordering, args...; init=init, kwargs...)
end
