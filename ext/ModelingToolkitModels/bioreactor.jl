"""The **photobioreactor–digester system problem** is a benchmark in constrained optimal control.

Builds the `bioreactor` problem with the ModelingToolkit backend.

```julia-repl
julia> using ModelingToolkit, OptimalControlProblems
julia> pb = bioreactor(ModelingToolkitBackend())
```

Default parameters and the modeling status are available through
`problem_metadata(:bioreactor)` and `OptimalControlProblems.status(:bioreactor)`.
"""
function OptimalControlProblems.bioreactor(
    ::ModelingToolkitBackend;
    parameters::Union{Nothing,NamedTuple}=nothing,
    init=nothing,
)
    OptimalControlProblems.Catalogue._warn_if_draft(:bioreactor, :ModelingToolkit)
    params = OptimalControlProblems.parameters_data(:bioreactor, parameters)
    t0, tf = float(params[:t0]), float(params[:tf])
    β, c, γ = params[:β], params[:c], params[:γ]
    halfperiod, Ks, μ2m, μbar, r = (
        params[:halfperiod], params[:Ks], params[:μ2m], params[:μbar], params[:r]
    )
    y_l, s_l, b_l = float(params[:y_l]), float(params[:s_l]), float(params[:b_l])
    u_l, u_u = float(params[:u_l]), float(params[:u_u])
    y_t0_l, y_t0_u = float(params[:y_t0_l]), float(params[:y_t0_u])
    s_t0_l, s_t0_u = float(params[:s_t0_l]), float(params[:s_t0_u])
    b_t0_l, b_t0_u = float(params[:b_t0_l]), float(params[:b_t0_u])

    t = ModelingToolkit.t_nounits
    D = ModelingToolkit.Differential(t)

    # the initial state is free, in a box: no default, started from the initial guess
    ModelingToolkit.@variables y(..) [tunable = true]
    ModelingToolkit.@variables s(..) [tunable = true]
    ModelingToolkit.@variables b(..) [tunable = true]
    ModelingToolkit.@variables u(..) [input = true, bounds = (u_l, u_u)]

    # light model: max(0, sin)^2 * μbar, on a day/night cycle of period 2 halfperiod
    days = t / (halfperiod * 2)
    tau = (days - floor(days)) * 2π
    μ = ifelse(sin(tau) > 0, sin(tau), 0)^2 * μbar
    # growth model (Monod)
    μ2 = μ2m * s(t) / (s(t) + Ks)

    eqs = [
        D(y(t)) ~ μ * y(t) / (1 + y(t)) - (r + u(t)) * y(t),
        D(s(t)) ~ -μ2 * b(t) + u(t) * β * (γ * y(t) - s(t)),
        D(b(t)) ~ (μ2 - u(t) * β) * b(t),
    ]
    costs = [ModelingToolkit.Symbolics.Integral(t in (t0, tf))(-μ2 * b(t) / (β + c))]
    constraints = [
        ModelingToolkit.Symbolics.:≳(y(t), y_l),
        ModelingToolkit.Symbolics.:≳(s(t), s_l),
        ModelingToolkit.Symbolics.:≳(b(t), b_l),
        ModelingToolkit.Symbolics.:≳(y(t0), y_t0_l),
        ModelingToolkit.Symbolics.:≲(y(t0), y_t0_u),
        ModelingToolkit.Symbolics.:≳(s(t0), s_t0_l),
        ModelingToolkit.Symbolics.:≲(s(t0), s_t0_u),
        ModelingToolkit.Symbolics.:≳(b(t0), b_t0_l),
        ModelingToolkit.Symbolics.:≲(b(t0), b_t0_u),
    ]

    ModelingToolkit.@named sys = ModelingToolkit.System(
        eqs, t; costs=costs, constraints=constraints
    )

    # initial guess: from the metadata (#377), unless given
    init = OptimalControlProblems.Metadata._initial_guess(:bioreactor, params, init)

    return OptimalControlProblems.Problems._problem(
        :bioreactor, :ModelingToolkit, sys, init, params
    )
end
