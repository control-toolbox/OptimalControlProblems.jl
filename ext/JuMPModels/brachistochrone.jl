"""
$(TYPEDSIGNATURES)

Constructs a JuMP model representing the **Brachistochrone optimal control problem**.
The objective is to minimize the final time `tf` to travel between two points under gravity.
The problem uses a direct transcription (trapezoidal rule) manually implemented in JuMP.

# Arguments

- `::JuMPBackend`: Placeholder type to specify the JuMP backend or solver interface.
- `grid_size::Int=50`: (Keyword) Number of discretisation steps for the time grid.

# Returns

- `model::JuMP.Model`: A JuMP model containing the decision variables (including final time), dynamics constraints, and boundary conditions.

# Example

```julia-repl
julia> using OptimalControlProblems
julia> using JuMP
julia> model = OptimalControlProblems.brachistochrone(JuMPBackend(); N=100)
```

# References

- Dymos Brachistochrone: [https://openmdao.github.io/dymos/examples/brachistochrone/brachistochrone.html](https://openmdao.github.io/dymos/examples/brachistochrone/brachistochrone.html)
"""
function OptimalControlProblems.brachistochrone(
    ::JuMPBackend,
    args...;
    grid_size::Int=grid_size_data(:brachistochrone),
    parameters::Union{Nothing,NamedTuple}=nothing,
    kwargs...,
)

    # parameters
    params = parameters_data(:brachistochrone, parameters)
    g = params[:g]
    t0 = params[:t0]
    x0 = params[:x0]
    y0 = params[:y0]
    v0 = params[:v0]
    xf = params[:xf]
    yf = params[:yf]
    u_min = params[:u_min]
    u_max = params[:u_max]

    # model
    model = JuMP.Model(args...; kwargs...)

    # N = grid_size
    @expression(model, N, grid_size)

    @variables(
        model,
        begin
            0.1 <= tf <= 20.0, (start = 2.0)
            px[0:N]
            py[0:N]
            v[0:N]
            -1.57 <= u[0:N] <= 1.57
        end
    )

    # metadata: required
    model[:time_grid] = () -> range(t0, value(tf), grid_size+1)
    model[:state_components] = ["px", "py", "v"]
    model[:costate_components] = ["∂px", "∂py", "∂v"]
    model[:control_components] = ["u"]
    model[:variable_components] = ["tf"]

    for i in 0:N
        alpha = i / N
        set_start_value(px[i], x0 + alpha * (xf - x0))
        set_start_value(py[i], y0 + alpha * (yf - y0))
        set_start_value(v[i], v0 + alpha * 10.0) # Estimate speed
        set_start_value(u[i], 1.57) # ~90 degrees
    end

    # boundary constraints
    @constraints(
        model,
        begin
            # Start
            px[0] == x0
            py[0] == y0
            v[0] == v0

            # End
            px[N] == xf
            py[N] == yf
            # v[N] is free
        end
    )

    # dynamics
    @expressions(
        model,
        begin
            # Time step is variable: dt = (tf - t0) / N
            dt, (tf - t0) / N

            # Dynamics expressions (Dymos formulation)
            # dpx/dt = v * sin(u)
            dpx[i = 0:N], v[i] * sin(u[i])

            # dpy/dt = -v * cos(u)
            dpy[i = 0:N], -v[i] * cos(u[i])

            # dv/dt = g * cos(u)
            dv[i = 0:N], g * cos(u[i])
        end
    )

    # Trapezoidal rule integration (Implicit)
    @constraints(
        model,
        begin
            ∂px[i = 1:N], px[i] == px[i - 1] + 0.5 * dt * (dpx[i] + dpx[i - 1])
            ∂py[i = 1:N], py[i] == py[i - 1] + 0.5 * dt * (dpy[i] + dpy[i - 1])
            ∂v[i = 1:N], v[i] == v[i - 1] + 0.5 * dt * (dv[i] + dv[i - 1])
        end
    )

    # objective: Minimize final time
    @objective(model, Min, tf)

    return model
end
