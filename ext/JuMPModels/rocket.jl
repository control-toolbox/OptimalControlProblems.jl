"""The **Goddard rocket problem** is a classical optimal control problem that models the ascent of a vertically launched rocket.

Builds the `rocket` problem with the JuMP backend.

```julia-repl
julia> using OptimalControlProblems
julia> rocket(JuMPBackend(); grid_size=100)
```

Default parameters and the modeling status are available through
`problem_metadata(:rocket)` and `OptimalControlProblems.status(:rocket)`.
"""
function OptimalControlProblems.rocket(
    ::JuMPBackend,
    args...;
    grid_size::Int=grid_size_data(:rocket),
    parameters::Union{Nothing,NamedTuple}=nothing,
    kwargs...,
)
    OptimalControlProblems._warn_if_draft(:rocket, :JuMP)

    # parameters
    params = parameters_data(:rocket, parameters)
    t0 = params[:t0]
    h_t0 = params[:h_t0]
    v_t0 = params[:v_t0]
    m_t0 = params[:m_t0]
    g0 = params[:g0]
    Tc = params[:Tc]
    hc = params[:hc]
    vc = params[:vc]
    mc = params[:mc]
    T_l = params[:T_l]
    tf_l = params[:tf_l]

    #
    c = 0.5 * sqrt(g0 * h_t0)
    m_tf = mc * m_t0
    Dc = 0.5 * vc * (m_t0 / g0)
    Tmax = Tc * m_t0 * g0

    function dynamics(x, u)
        h, v, m = x
        T = u[1]
        D = Dc * v^2 * exp(-hc * (h - h_t0)) / h_t0
        g = g0 * (h_t0 / h)^2
        return [v, (T - D - m * g) / m, -T / c]
    end

    N = grid_size
    data = ProblemData(;
        state=[
            Component("h"; lower=h_t0, start=1),
            Component("v"; lower=v_t0, start=[i / N * (1 - i / N) for i in 0:N]),
            Component("m"; lower=m_tf, upper=m_t0, start=[(m_tf - m_t0) * (i / N) + m_t0 for i in 0:N]),
        ],
        control=[Component("T"; lower=T_l, upper=Tmax, start=Tmax / 2)],
        variable=[Component("tf"; lower=tf_l, start=1)],
        costate=["∂h", "∂v", "∂m"],
        times=v -> (t0, v[1]),
        dynamics=(x, u, v, t) -> dynamics(x, u),
        cost=(I, x0, xf, v) -> -xf[1],
        boundary=[
            (h_t0, (x0, xf, v) -> x0[1], h_t0),
            (v_t0, (x0, xf, v) -> x0[2], v_t0),
            (m_t0, (x0, xf, v) -> x0[3], m_t0),
            (m_tf, (x0, xf, v) -> xf[3], m_tf),
        ],
    )
    return build_trapeze(data, grid_size, args...; kwargs...)
end
