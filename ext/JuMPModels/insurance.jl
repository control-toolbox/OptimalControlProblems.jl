"""The **insurance problem** is a benchmark in optimal control inspired by health and financial economics.

Builds the `insurance` problem with the JuMP backend.

```julia-repl
julia> using OptimalControlProblems
julia> insurance(JuMPBackend(); grid_size=100)
```

The keyword `scheme` chooses the discretization: `:trapeze` (default), `:euler`,
`:euler_implicit` or `:midpoint`, as in `OptimalControl.Collocation`.
The keyword `ordering` chooses the order of the variables and constraints:
`:interleaved` (default, as in CTDirect) or `:blocks` (grouped by kind).

Default parameters and the modeling status are available through
`problem_metadata(:insurance)` and `OptimalControlProblems.status(:insurance)`.
"""
function OptimalControlProblems.insurance(
    ::JuMPBackend,
    args...;
    grid_size::Int=grid_size_data(:insurance),
    scheme::Symbol=:trapeze,
    ordering::Symbol=:interleaved,
    parameters::Union{Nothing,NamedTuple}=nothing,
    init=nothing,
    kwargs...,
)
    OptimalControlProblems.Catalogue._warn_if_draft(:insurance, :JuMP)

    # parameters
    params = parameters_data(:insurance, parameters)
    t0 = params[:t0]
    tf = params[:tf]
    γ = params[:γ]
    λ = params[:λ]
    h0 = params[:h0]
    w = params[:w]
    s = params[:s]
    k = params[:k]
    σ = params[:σ]
    α = params[:α]
    I_l = params[:I_l]
    I_u = params[:I_u]
    m_l = params[:m_l]
    m_u = params[:m_u]
    h_l = params[:h_l]
    h_u = params[:h_u]
    R_l = params[:R_l]
    H_l = params[:H_l]
    U_l = params[:U_l]
    dUdR_l = params[:dUdR_l]
    P_l = params[:P_l]
    I_t0 = params[:I_t0]
    m_t0 = params[:m_t0]
    x₃_t0 = params[:x₃_t0]

    fx(t) = λ * exp(-λ * (t - t0)) + exp(-λ * (tf - t0)) / (tf - t0)

    function dynamics(x, u, t)
        I, m, _ = x
        h, _, _, _, dUdR = u
        vprime = α / 2 * m^(α / 2 - 1) / (1 + m^(α / 2))^2
        return [(1 - γ * (t - t0) * vprime / dUdR) * h, h, (1 + σ) * I * fx(t)]
    end

    # algebraic relations between the controls, imposed at every node
    ε(t) = k * (t - t0) / (tf - t + 1)
    v(m) = m^(α / 2) / (1 + m^(α / 2))

    data = ProblemData(;
        state=[
            Component("I"; lower=I_l, upper=I_u),
            Component("m"; lower=m_l, upper=m_u),
            Component("x₃"),
        ],
        control=[
            Component("h"; lower=h_l, upper=h_u),
            Component("R"; lower=R_l),
            Component("H"; lower=H_l),
            Component("U"; lower=U_l),
            Component("dUdR"; lower=dUdR_l),
        ],
        variable=[Component("P"; lower=P_l)],
        costate=["∂I", "∂m", "∂x₃"],
        times=v -> (t0, tf),
        dynamics=(x, u, v, t) -> dynamics(x, u, t),
        lagrange=(x, u, v, t) -> u[4] * fx(t),
        cost=(integral, x0, xf, v) -> -integral,
        path=[
            (0, (x, u, P, t) -> u[2] - (w - P[1] + x[1] - x[2] - ε(t)), 0),
            (0, (x, u, P, t) -> u[3] - (h0 - γ * (t - t0) * (1 - v(x[2]))), 0),
            (0, (x, u, P, t) -> u[4] - (1 - exp(-s * u[2]) + u[3]), 0),
            (0, (x, u, P, t) -> u[5] - (s * exp(-s * u[2])), 0),
        ],
        boundary=[
            (I_t0, (x0, xf, v) -> x0[1], I_t0),
            (m_t0, (x0, xf, v) -> x0[2], m_t0),
            (x₃_t0, (x0, xf, v) -> x0[3], x₃_t0),
            (0, (x0, xf, v) -> v[1] - xf[3], 0),
        ],
    )
    # initial guess: from the metadata (#377), unless given
    init = OptimalControlProblems.Metadata._initial_guess(:insurance, params, init)
    return build(data, grid_size, scheme, ordering, args...; init=init, kwargs...)
end
