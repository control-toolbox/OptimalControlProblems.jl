"""
$(TYPEDSIGNATURES)

Constructs and returns a JuMP model for the **Goddard Rocket Optimal Control Problem**.  
The model represents the dynamics of a Goddard rocket with three states (altitude `h`, velocity `v`, and mass `m`) and one control input (`T` for thrust).  
The objective is to maximise the final altitude of the rocket while satisfying boundary conditions and dynamic constraints.

# Arguments

- `::JuMPBackend`: Specifies the backend for building the JuMP model.
- `N::Int=500`: (Keyword) Number of discretisation steps for the time horizon.

# Returns

- `model::JuMP.Model`: A JuMP model representing the Goddard rocket optimal control problem.

# Example

```julia-repl
julia> using OptimalControlProblems
julia> using JuMP

julia> model = OptimalControlProblems.rocket(JuMPBackend(); N=200)
```

# References

- Problem formulation available at: https://github.com/MadNLP/COPSBenchmark.jl/blob/main/src/rocket.jl
"""
function OptimalControlProblems.rocket(::JuMPBackend; N::Int=500)

    # parameters
    h0 = 1
    v0 = 0
    m0 = 1
    g0 = 1
    Tc = 3.5
    hc = 500
    vc = 620
    mc = 0.6
    c = 0.5 * sqrt(g0 * h0)
    mf = mc * m0
    Dc = 0.5 * vc * (m0 / g0)
    Tmax = Tc * m0 * g0

    # model
    model = JuMP.Model()

    # state, control, variable (final time) and initial guess
    @variables(
        model,
        begin
            h[i = 0:N] >= h0, (start = 1)
            v[i = 0:N] >= v0, (start = i / N * (1 - i / N))
            mf <= m[i = 0:N] <= m0, (start = (mf - m0) * (i / N) + m0)
            0 <= T[i = 0:N] <= Tmax, (start = Tmax / 2)
            0 <= tf, (start = 1)
        end
    )

    # boundary constraints
    @constraints(
        model,
        begin
            h_ic, h[0] == h0
            v_ic, v[0] == v0
            m_ic, m[0] == m0
            mfc, m[N] == mf
        end
    )

    # dynamics
    @expressions(
        model,
        begin

            #
            step, tf / N

            #
            D[i = 0:N], Dc * v[i]^2 * exp(-hc * (h[i] - h0)) / h0
            g[i = 0:N], g0 * (h0 / h[i])^2

            #
            dh[i = 0:N], v[i]
            dv[i = 0:N], (T[i] - D[i] - m[i] * g[i]) / m[i]
            dm[i = 0:N], -T[i] / c
        end
    )

    @constraints(
        model,
        begin
            ∂h[i = 1:N], h[i] == h[i - 1] + 0.5 * step * (dh[i] + dh[i - 1])
            ∂v[i = 1:N], v[i] == v[i - 1] + 0.5 * step * (dv[i] + dv[i - 1])
            ∂m[i = 1:N], m[i] == m[i - 1] + 0.5 * step * (dm[i] + dm[i - 1])
        end
    )

    # objective
    @objective(model, Min, -h[N])

    return model
end
