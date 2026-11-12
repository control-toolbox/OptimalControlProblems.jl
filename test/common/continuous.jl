# Comparison of the modelings of a problem at the continuous level (#441): the JuMP `ProblemData`
# of a problem (`model[:problem_data]`) against the OptimalControl model, without discretizing.
# Used by `suite/comparison/test_modelings`. Do they describe the same problem? The times, the
# dynamics, the objective, the path and boundary constraints and the bounds are evaluated at the
# same points, taken on the initial guess of the problem (the solvers start there, so the
# functions are defined there) and perturbed. Cheap, whatever the problem: nothing is compiled
# for a discretization, an NLP or a solver.
module Continuous

using OptimalControl: OptimalControl

const OC = OptimalControl

# The perturbations of the initial guess, and the points of each trajectory (fractions of the
# time interval): the objective is not affine, several trajectories tell two objectives apart.
const AMPLITUDES = (0.0, 0.01, 0.05)
const FRACTIONS = (0.0, 0.25, 0.5, 0.75, 1.0)
const TOL = 1e-10

vec1(a::Number) = [Float64(a)]
vec1(a) = Float64.(collect(a))

# Equal, or equal to the tolerance; infinite bounds are equal when they are the same.
function close_to(a, b)
    length(a) == length(b) || return false
    return all(@. (a == b) | (abs(a - b) <= TOL * max(1, abs(a), abs(b))))
end

"""Trapezoidal rule of the values `y` of a function at the times `t`: the quadrature of both
modelings, so that their objectives are compared on the same integral."""
function quadrature(t, y)
    return sum(
        (t[i + 1] - t[i]) * (y[i] + y[i + 1]) / 2 for i in 1:(length(t) - 1); init=0.0
    )
end

guess(g, t) = g isa Function ? g(t) : g

"""
The trajectories on which the problems are compared: the initial guess `init`, perturbed by
each of `AMPLITUDES`, at the times `FRACTIONS` of `(t0, tf)`. A trajectory is `(t, x, u)`, the
variable `v` being the one of the initial guess (`1` when there is none).
"""
function trajectories(init, n, m, t0, tf)
    t = [t0 + s * (tf - t0) for s in FRACTIONS]
    return map(AMPLITUDES) do a
        shift(k, i) = a * (1 + 0.1 * i + 0.3 * k)    # positive, different at each node and component
        x = [vec1(haskey(init, :state) ? guess(init.state, s) : fill(0.5, n)) for s in t]
        u = [
            vec1(haskey(init, :control) ? guess(init.control, s) : fill(0.5, m)) for s in t
        ]
        x = [x[k] .+ shift.(k, 1:n) for k in eachindex(t)]
        u = [u[k] .+ shift.(k, 1:m) for k in eachindex(t)]
        return (; t, x, u)
    end
end

# -----------------------------------------------------------------------------
# The OptimalControl model as plain functions of the point. Its dynamics and constraints are
# in place: `f!(r, t, x, u, v)`, and `f!(r, x0, xf, v)` for the boundary conditions.
# -----------------------------------------------------------------------------
function dynamics(ocp)
    f!, n = OC.dynamics(ocp), OC.state_dimension(ocp)
    return (t, x, u, v) -> (r=zeros(n); f!(r, t, x, u, v); r)
end

function path(ocp)
    lb, f!, ub, _ = OC.path_constraints_nl(ocp)
    return lb, ub, (t, x, u, v) -> (r=zeros(length(lb)); f!(r, t, x, u, v); r)
end

function boundary(ocp)
    lb, f!, ub, _ = OC.boundary_constraints_nl(ocp)
    return lb, ub, (x0, xf, v) -> (r=zeros(length(lb)); f!(r, x0, xf, v); r)
end

"""Bounds `(lower, upper)` of each of the `dim` components, from the box constraints `box` of
OptimalControl; `±Inf` where a component has none."""
function box(box, dim)
    lower, upper = fill(-Inf, dim), fill(Inf, dim)
    for (l, i, u) in zip(box[1], box[2], box[3])
        lower[i] = max(lower[i], l)
        upper[i] = min(upper[i], u)
    end
    return lower, upper
end

"""Bounds `(lower, upper)` of each component of the JuMP model; a bound given by node (a
vector) is not compared, which `compare` reports as a difference."""
function bounds(components)
    bound(b, default) = isnothing(b) ? default : b
    return [bound(c.lower, -Inf) for c in components],
    [bound(c.upper, Inf) for c in components]
end

"""
Key of a constraint: its bounds and its values at the points. Constraints are matched by their
keys, not by their order, which the two modelings do not share.
"""
key(l, u, values) = Tuple(round.([l; u; values]; sigdigits=8) .+ 0.0)   # + 0.0: -0.0 and 0.0 agree

"""
    compare(data, ocp, init)

The checks of the JuMP `ProblemData` `data` against the OptimalControl model `ocp` of the same
problem, `init` being its initial guess: a `Dict` from the name of a check to whether it holds.
"""
function compare(data, ocp, init)
    n, m, nv = OC.state_dimension(ocp),
    OC.control_dimension(ocp),
    OC.variable_dimension(ocp)
    v = haskey(init, :variable) ? vec1(init.variable) : ones(nv)
    ok = Dict{Symbol,Bool}()

    # times: fixed, or a function of the variable
    t0 = OC.has_fixed_initial_time(ocp) ? OC.initial_time(ocp) : OC.initial_time(ocp, v)
    tf = OC.has_fixed_final_time(ocp) ? OC.final_time(ocp) : OC.final_time(ocp, v)
    ok[:times] = close_to([t0, tf], collect(data.times(v)))
    trajs = trajectories(init, n, m, t0, tf)
    points = [(t=tr.t[k], x=tr.x[k], u=tr.u[k]) for tr in trajs for k in eachindex(tr.t)]

    # dynamics
    f = dynamics(ocp)
    ok[:dynamics] = all(
        p -> close_to(f(p.t, p.x, p.u, v), vec1(data.dynamics(p.x, p.u, v, p.t))), points
    )

    # objective, on each trajectory. JuMP minimizes `cost(I, x0, xf, v)` where `I` is the
    # integral of its Lagrange integrand; OptimalControl minimizes (`:min`) or maximizes (`:max`)
    # `mayer + ∫ lagrange`. The two write the same objective with a different split (the
    # OptimalControl parser puts a sign or a factor into the integrand): the objectives are
    # compared, not the integrands.
    sign = OC.criterion(ocp) === :min ? 1.0 : -1.0
    L = OC.has_lagrange_cost(ocp) ? OC.lagrange(ocp) : nothing
    g = OC.has_mayer_cost(ocp) ? OC.mayer(ocp) : nothing
    ok[:objective] = all(trajs) do tr
        integrand_jump = if isnothing(data.lagrange)
            0.0
        else
            quadrature(
            tr.t, [data.lagrange(tr.x[k], tr.u[k], v, tr.t[k]) for k in eachindex(tr.t)]
        )
        end
        integrand_oc = if isnothing(L)
            0.0
        else
            quadrature(tr.t, [L(tr.t[k], tr.x[k], tr.u[k], v) for k in eachindex(tr.t)])
        end
        mayer = isnothing(g) ? 0.0 : g(tr.x[1], tr.x[end], v)
        return close_to(
            [data.cost(integrand_jump, tr.x[1], tr.x[end], v)],
            [sign * (mayer + integrand_oc)],
        )
    end

    # path constraints, matched by their bounds and values
    lb, ub, c = path(ocp)
    oc_keys = [
        key(lb[i], ub[i], [c(p.t, p.x, p.u, v)[i] for p in points]) for i in eachindex(lb)
    ]
    jump_keys = [
        key(l, u, [first(vec1(h(p.x, p.u, v, p.t))) for p in points]) for
        (l, h, u) in data.path
    ]
    ok[:path] = sort(oc_keys) == sort(jump_keys)

    # boundary conditions, matched by their bounds and values at the ends of each trajectory
    lb, ub, c = boundary(ocp)
    ends = [(tr.x[1], tr.x[end]) for tr in trajs]
    oc_keys = [
        key(lb[i], ub[i], [c(x0, xf, v)[i] for (x0, xf) in ends]) for i in eachindex(lb)
    ]
    jump_keys = [
        key(l, u, [first(vec1(h(x0, xf, v))) for (x0, xf) in ends]) for
        (l, h, u) in data.boundary
    ]
    ok[:boundary] = sort(oc_keys) == sort(jump_keys)

    # bounds of the state, the control and the variable
    for (name, components, dim, b) in (
        (:state_bounds, data.state, n, OC.state_constraints_box(ocp)),
        (:control_bounds, data.control, m, OC.control_constraints_box(ocp)),
        (:variable_bounds, data.variable, nv, OC.variable_constraints_box(ocp)),
    )
        lower, upper = box(b, dim)
        jump_lower, jump_upper = bounds(components)
        by_node =
            any(x -> x isa AbstractVector, jump_lower) ||
            any(x -> x isa AbstractVector, jump_upper)
        ok[name] = !by_node && close_to(lower, jump_lower) && close_to(upper, jump_upper)
    end
    return ok
end

end # module Continuous
