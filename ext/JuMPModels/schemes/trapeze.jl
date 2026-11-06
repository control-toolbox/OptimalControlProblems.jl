# Trapezoidal scheme (#368), in CTDirect's order: the variables node by node (state, then
# control), then the variable; for each step the dynamics then the path constraints at its
# first node, then the path constraints at the last node, then the boundary conditions.

_at(a, i) = a isa AbstractVector ? a[i + 1] : a            # value at node i (0-based)
_value(a) = a isa Number ? a : JuMP.value(a)

# One JuMP variable, named as a container element (`x₁[3]`) or a scalar (`tf`).
function _variable(model, c::Component, name, i)
    x = JuMP.@variable(model, base_name=name)
    lower = isnothing(i) ? c.lower : _at(c.lower, i)
    upper = isnothing(i) ? c.upper : _at(c.upper, i)
    start = isnothing(i) ? c.start : _at(c.start, i)
    isnothing(lower) || JuMP.set_lower_bound(x, lower)
    isnothing(upper) || JuMP.set_upper_bound(x, upper)
    isnothing(start) || JuMP.set_start_value(x, start)
    return x
end

function _constraint(model, lower, f, upper)
    return if lower == upper
        JuMP.@constraint(model, f == lower)
    else
        JuMP.@constraint(model, lower <= f <= upper)
    end
end

"""
$(TYPEDSIGNATURES)

Build the JuMP model of `data` discretized by the trapezoidal rule on `grid_size` steps.
`args` and `kwargs` are passed to `JuMP.Model`.
"""
function build_trapeze(data::ProblemData, grid_size::Int, args...; kwargs...)
    N = grid_size
    model = JuMP.Model(args...; kwargs...)

    # variables, node by node
    X = Matrix{JuMP.VariableRef}(undef, length(data.state), N + 1)
    U = Matrix{JuMP.VariableRef}(undef, length(data.control), N + 1)
    for i in 0:N
        for (k, c) in enumerate(data.state)
            X[k, i + 1] = _variable(model, c, "$(c.name)[$i]", i)
        end
        for (k, c) in enumerate(data.control)
            U[k, i + 1] = _variable(model, c, "$(c.name)[$i]", i)
        end
    end
    v = [_variable(model, c, c.name, nothing) for c in data.variable]
    for (k, c) in enumerate(data.state)
        model[Symbol(c.name)] = JuMP.Containers.DenseAxisArray(X[k, :], 0:N)
    end
    for (k, c) in enumerate(data.control)
        model[Symbol(c.name)] = JuMP.Containers.DenseAxisArray(U[k, :], 0:N)
    end
    for (k, c) in enumerate(data.variable)
        model[Symbol(c.name)] = v[k]
    end

    # time grid
    t0, tf = data.times(v)
    Δt = (tf - t0) / N
    t = [t0 + i * Δt for i in 0:N]
    x = [X[:, i + 1] for i in 0:N]
    u = [U[:, i + 1] for i in 0:N]

    # dynamics and path constraints
    f = [data.dynamics(x[i + 1], u[i + 1], v, t[i + 1]) for i in 0:N]
    z = [data.state_expressions(x[i + 1], v) for i in 0:N]
    D = Matrix{JuMP.ConstraintRef}(undef, length(data.state), N)
    function path(i)
        for (lower, g, upper) in data.path
            _constraint(model, lower, g(x[i + 1], u[i + 1], v, t[i + 1]), upper)
        end
    end
    for i in 1:N
        for k in eachindex(data.state)
            D[k, i] = JuMP.@constraint(
                model, z[i + 1][k] == z[i][k] + 0.5 * Δt * (f[i + 1][k] + f[i][k])
            )
        end
        path(i - 1)
    end
    path(N)
    for (k, name) in enumerate(data.costate)
        model[Symbol(name)] = JuMP.Containers.DenseAxisArray(D[k, :], 1:N)
    end

    # boundary conditions
    for (lower, g, upper) in data.boundary
        _constraint(model, lower, g(x[1], x[N + 1], v), upper)
    end

    # objective
    I = if isnothing(data.lagrange)
        0
    else
        L = [data.lagrange(x[i + 1], u[i + 1], v, t[i + 1]) for i in 0:N]
        0.5 * Δt * sum(L[i + 1] + L[i] for i in 1:N)
    end
    JuMP.@objective(model, Min, data.cost(I, x[1], x[N + 1], v))

    # metadata read by the accessors
    model[:time_grid] = () -> range(_value(t0), _value(tf), N + 1)
    model[:state_components] = [c.name for c in data.state]
    model[:costate_components] = data.costate
    model[:control_components] = [c.name for c in data.control]
    model[:variable_components] = [c.name for c in data.variable]
    return model
end
