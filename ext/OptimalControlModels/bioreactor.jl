"""The **photobioreactor–digester system problem** is a benchmark in constrained optimal control.

Builds the `bioreactor` problem with the OptimalControl backend.

```julia-repl
julia> using OptimalControlProblems
julia> bioreactor(OptimalControlBackend())
```

Default parameters and the modeling status are available through
`problem_metadata(:bioreactor)` and `OptimalControlProblems.status(:bioreactor)`.
"""
function OptimalControlProblems.bioreactor(
    ::OptimalControlBackend;
    parameters::Union{Nothing,NamedTuple}=nothing,
    init=nothing,
)
    OptimalControlProblems.Catalogue._warn_if_draft(:bioreactor, :OptimalControl)

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

    # Model
    ocp = @def begin
        t ∈ [t0, tf], time
        x = (y, s, b) ∈ R³, state
        u ∈ R, control

        x(t) ≥ [y_l, s_l, b_l]
        u_l ≤ u(t) ≤ u_u
        [y_t0_l, s_t0_l, b_t0_l] ≤ x(t0) ≤ [y_t0_u, s_t0_u, b_t0_u]

        μ = light(t, halfperiod) * μbar
        μ2 = growth(s(t), μ2m, Ks)

        ẋ(t) == [
            μ * y(t) / (1 + y(t)) - (r + u(t)) * y(t),
            -μ2 * b(t) + u(t) * β * (γ * y(t) - s(t)),
            (μ2 - u(t) * β) * b(t),
        ]

        -∫(μ2 * b(t) / (β + c)) → min
    end

    # METHANE PROBLEM
    # μ2 according to growth model
    # μ according to light model
    # time scale is [0,10] for 24h (day then night)

    # growth model MONOD
    function growth(s, μ2m, Ks)
        return μ2m * s / (s + Ks)
    end

    # light model: max^2 (0,sin) * μbar
    # DAY/NIGHT CYCLE: [0,2 halfperiod] rescaled to [0,2pi]
    function light(time, halfperiod)
        days = time / (halfperiod * 2)
        tau = (days - floor(days)) * 2π
        return max(0, sin(tau))^2
    end

    # initial guess: from the metadata (#377), unless given
    init = OptimalControlProblems.Metadata._initial_guess(:bioreactor, params, init)

    return OptimalControlProblems.Problems._problem(:bioreactor, :OptimalControl, ocp, init, params)
end
