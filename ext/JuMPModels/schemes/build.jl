# The JuMP model of a problem for a scheme given by its Butcher tableau (#368, #369), in one
# of two orders (#370):
# - `:interleaved`, CTDirect's order: the variables node by node (state, then control while
#   there is one), then the variable; for each step the dynamics then the path constraints
#   at its first node, then the path constraints at the last node, then the boundary
#   conditions;
# - `:blocks`: the same pieces grouped by kind, all states, all controls, the variable; all
#   dynamics, all path constraints, the boundary conditions.

const ORDERINGS = (:interleaved, :blocks)

function _check_ordering(ordering::Symbol)
    ordering in ORDERINGS || throw(
        CTBase.Exceptions.IncorrectArgument(
            "Unknown ordering for the JuMP models";
            got=":$ordering",
            expected=join((":$o" for o in ORDERINGS), ", "),
            suggestion="use one of the orderings listed above",
            context="JuMP model of OptimalControlProblems",
        ),
    )
    return ordering
end

# Call the pieces `(kind, f)`, listed in CTDirect's order, in the order of `ordering`: as
# listed, or grouped by kind (a stable sort keeps CTDirect's order within a kind). The
# results come back in CTDirect's order.
function _in_order(pieces, ordering)
    order = ordering === :interleaved ? eachindex(pieces) : sortperm(first.(pieces))
    results = Vector{Any}(undef, length(pieces))
    for k in order
        results[k] = last(pieces[k])()
    end
    return results
end

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

# Stage `Z = X_i + θ (X_{i+1} - X_i)` of a step, and its time; a stage at a node is the node.
function _stage(xi, xj, θ)
    θ == 0 && return xi
    θ == 1 && return xj
    return [(1 - θ) * a + θ * b for (a, b) in zip(xi, xj)]
end
_stage_time(ti, tj, Δt, c) =
    if c == 0
        ti
    elseif c == 1
        tj
    else
        ti + c * Δt
    end

# Quadrature of the stage values `G` with the weights `b`.
function _weighted(b, G)
    return length(b) == 1 && b[1] == 1 ? G[1] : sum(b[j] * G[j] for j in eachindex(b))
end

"""
$(TYPEDSIGNATURES)

Values of `g(x, u, v, t)` at the stages of every step: `G[i + 1][j]` for stage `j` of step
`i`. With FSAL, the stages are nodes and `g` is evaluated once per node.
"""
function _stage_values(::FirstSameAsLast, g, tab, θ, x, u, v, t, Δt)
    N = length(x) - 1
    G = [g(x[k + 1], u[k + 1], v, t[k + 1]) for k in 0:N]
    return [[G[i + Int(θ[j]) + 1] for j in eachindex(θ)] for i in 0:(N - 1)]
end

function _stage_values(::NoStageReuse, g, tab, θ, x, u, v, t, Δt)
    N = length(x) - 1
    return map(0:(N - 1)) do i
        map(eachindex(θ)) do j
            Z = _stage(x[i + 1], x[i + 2], θ[j])
            U = u[control_at_stage(tab, θ, i, j) + 1]
            return g(Z, U, v, _stage_time(t[i + 1], t[i + 2], Δt, tab.c[j]))
        end
    end
end

"""
$(TYPEDSIGNATURES)

Build the JuMP model of `data` discretized on `grid_size` steps by `scheme` (`:trapeze`,
`:euler`, `:euler_implicit` or `:midpoint`), with the variables and constraints in the
order `ordering` (`:interleaved` or `:blocks`). `args` and `kwargs` are passed to
`JuMP.Model`.

# Throws

- `CTBase.Exceptions.IncorrectArgument`: If `scheme` or `ordering` is not one of them.
"""
function build(
    data::ProblemData, grid_size::Int, scheme::Symbol, ordering::Symbol, args...; kwargs...
)
    tab = tableau(scheme)
    _check_ordering(ordering)
    θ = stage_positions(tab)
    N = grid_size
    M = control_count(tab, N)
    model = JuMP.Model(args...; kwargs...)

    # variables: the state at each node (kind 1), the control of each node while there is
    # one (kind 2), in CTDirect's order; then the variable
    X = Matrix{JuMP.VariableRef}(undef, length(data.state), N + 1)
    U = Matrix{JuMP.VariableRef}(undef, length(data.control), M)
    function variables!(V, components, i)
        for (k, c) in enumerate(components)
            V[k, i + 1] = _variable(model, c, "$(c.name)[$i]", i)
        end
    end
    nodes = Tuple{Int,Function}[]
    for i in 0:N
        push!(nodes, (1, () -> variables!(X, data.state, i)))
        i < M && push!(nodes, (2, () -> variables!(U, data.control, i)))
    end
    _in_order(nodes, ordering)
    v = [_variable(model, c, c.name, nothing) for c in data.variable]
    for (k, c) in enumerate(data.state)
        model[Symbol(c.name)] = JuMP.Containers.DenseAxisArray(X[k, :], 0:N)
    end
    for (k, c) in enumerate(data.control)
        model[Symbol(c.name)] = JuMP.Containers.DenseAxisArray(U[k, :], 0:(M - 1))
    end
    for (k, c) in enumerate(data.variable)
        model[Symbol(c.name)] = v[k]
    end

    # time grid
    t0, tf = data.times(v)
    Δt = (tf - t0) / N
    t = [t0 + i * Δt for i in 0:N]
    x = [X[:, i + 1] for i in 0:N]
    u = [U[:, i + 1] for i in 0:(M - 1)]

    # dynamics and path constraints
    reuse = stage_reuse(tab)
    F = _stage_values(reuse, data.dynamics, tab, θ, x, u, v, t, Δt)
    z = [data.state_expressions(x[i + 1], v) for i in 0:N]
    D = Matrix{JuMP.ConstraintRef}(undef, length(data.state), N)
    function dynamics(i)
        f = _weighted(tab.b, F[i])
        for k in eachindex(data.state)
            D[k, i] = JuMP.@constraint(model, z[i + 1][k] == z[i][k] + Δt * f[k])
        end
        return D[:, i]
    end
    function path(k)
        uk = u[control_at_node(tab, k, N) + 1]
        return [
            _constraint(model, lower, g(x[k + 1], uk, v, t[k + 1]), upper) for
            (lower, g, upper) in data.path
        ]
    end
    boundary() = [
        _constraint(model, lower, g(x[1], x[N + 1], v), upper) for
        (lower, g, upper) in data.boundary
    ]

    # constraints: the dynamics of each step (kind 1), the path constraints at each node
    # (kind 2), the boundary conditions (kind 3), in CTDirect's order
    pieces = Tuple{Int,Function}[]
    for i in 1:N
        push!(pieces, (1, () -> dynamics(i)), (2, () -> path(i - 1)))
    end
    push!(pieces, (2, () -> path(N)), (3, boundary))
    model[:constraints] = reduce(
        vcat, _in_order(pieces, ordering); init=JuMP.ConstraintRef[]
    )
    for (k, name) in enumerate(data.costate)
        model[Symbol(name)] = JuMP.Containers.DenseAxisArray(D[k, :], 1:N)
    end

    # objective
    I = if isnothing(data.lagrange)
        0
    else
        L = _stage_values(reuse, data.lagrange, tab, θ, x, u, v, t, Δt)
        Δt * sum(_weighted(tab.b, L[i]) for i in 1:N)
    end
    JuMP.@objective(model, Min, data.cost(I, x[1], x[N + 1], v))

    # metadata read by the accessors
    model[:scheme] = scheme
    model[:ordering] = ordering
    model[:time_grid] = () -> range(_value(t0), _value(tf), N + 1)
    model[:state_components] = [c.name for c in data.state]
    model[:costate_components] = data.costate
    model[:control_components] = [c.name for c in data.control]
    model[:variable_components] = [c.name for c in data.variable]
    return model
end
