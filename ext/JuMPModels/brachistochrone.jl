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
    g  = params[:g]
    t0 = params[:t0]
    x0 = params[:x0]
    y0 = params[:y0]
    v0 = params[:v0]
    xf = params[:xf]
    yf = params[:yf]

    # model
    model = JuMP.Model(args...; kwargs...)

    # N = grid_size
    @expression(model, N, grid_size)

    @variables(
        model,
        begin
            0.1 <= tf <= 20.0, (start = 2.0)
            x[0:N]
            y[0:N]
            v[0:N]
            u[0:N]
        end
    )


    model[:time_grid] = () -> range(t0, value(tf), N+1)
    model[:state_components] = ["x", "y", "v"]
    
    # --- CORRECTION ICI ---
    # Il faut déclarer la liste des costates (vide ici) pour satisfaire les tests
    model[:costate_components] = String[]
    # ----------------------
    
    model[:control_components] = ["u"]
    model[:variable_components] = ["tf"]

    for i in 0:N
        alpha = i / N
        set_start_value(x[i], x0 + alpha * (xf - x0))
        set_start_value(y[i], y0 + alpha * (yf - y0))
        set_start_value(v[i], v0 + alpha * 10.0) # Estimate speed
        set_start_value(u[i], 1.57) # ~90 degrees
    end

    # boundary constraints
    @constraints(
        model,
        begin
            # Start
            x[0] == x0
            y[0] == y0
            v[0] == v0
            
            # End
            x[N] == xf
            y[N] == yf
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
            # dx/dt = v * sin(u)
            dx[i = 0:N], v[i] * sin(u[i])
            
            # dy/dt = v * cos(u)
            dy[i = 0:N], v[i] * cos(u[i])
            
            # dv/dt = g * cos(u)
            dv[i = 0:N], g * cos(u[i])
        end
    )

    # Trapezoidal rule integration (Implicit)
    @constraints(
        model,
        begin
            ∂x[i = 1:N], x[i] == x[i - 1] + 0.5 * dt * (dx[i] + dx[i - 1])
            ∂y[i = 1:N], y[i] == y[i - 1] + 0.5 * dt * (dy[i] + dy[i - 1])
            ∂v[i = 1:N], v[i] == v[i - 1] + 0.5 * dt * (dv[i] + dv[i - 1])
        end
    )

    # objective: Minimize final time
    @objective(model, Min, tf)

    return model
end