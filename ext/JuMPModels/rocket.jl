"""The **Goddard rocket problem** is a classical optimal control problem that models the ascent of a vertically launched rocket.

Builds the `rocket` problem with the JuMP backend.

```julia-repl
julia> using OptimalControlProblems
julia> rocket(JuMPBackend(); grid_size=100)
```

Default parameters and the modeling status are available through `metadata(:rocket)` and
`status(:rocket)`.
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

    # model
    model = JuMP.Model(args...; kwargs...)

    # metadata
    model[:time_grid] = () -> range(t0, value(model[:tf]), grid_size + 1) # tf is a free
    model[:state_components] = ["h", "v", "m"]
    model[:costate_components] = ["∂h", "∂v", "∂m"]
    model[:control_components] = ["T"]
    model[:variable_components] = ["tf"]

    # N = grid_size
    @expression(model, N, grid_size)

    # state, control, variable (final time) and initial guess
    @variables(
        model,
        begin
            h[i = 0:N] ≥ h_t0, (start = 1)
            v[i = 0:N] ≥ v_t0, (start = i / N * (1 - i / N))
            m_tf ≤ m[i = 0:N] ≤ m_t0, (start = (m_tf - m_t0) * (i / N) + m_t0)
            T_l ≤ T[i = 0:N] ≤ Tmax, (start = Tmax / 2)
            tf ≥ tf_l, (start = 1)
        end
    )

    # boundary constraints
    @constraints(
        model,
        begin
            h[0] == h_t0
            v[0] == v_t0
            m[0] == m_t0
            m[N] == m_tf
        end
    )

    # dynamics
    @expressions(
        model,
        begin
            #
            Δt, (tf - t0) / N

            #
            D[i = 0:N], Dc * v[i]^2 * exp(-hc * (h[i] - h_t0)) / h_t0
            g[i = 0:N], g0 * (h_t0 / h[i])^2

            #
            dh[i = 0:N], v[i]
            dv[i = 0:N], (T[i] - D[i] - m[i] * g[i]) / m[i]
            dm[i = 0:N], -T[i] / c
        end
    )

    @constraints(
        model,
        begin
            ∂h[i = 1:N], h[i] == h[i - 1] + 0.5 * Δt * (dh[i] + dh[i - 1])
            ∂v[i = 1:N], v[i] == v[i - 1] + 0.5 * Δt * (dv[i] + dv[i - 1])
            ∂m[i = 1:N], m[i] == m[i - 1] + 0.5 * Δt * (dm[i] + dm[i - 1])
        end
    )

    # objective
    @objective(model, Min, -h[N])

    return model
end
