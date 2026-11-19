# The ModelingToolkit model of a problem as the continuous data `Continuous.compare` reads
# (#439): the fields of the JuMP `ProblemData` (times, dynamics, Lagrange integrand, cost, path
# and boundary constraints, bounds), evaluated from the `System` at numerical points, without
# compiling it. Used by `suite/ModelingToolkit/test_modelings`, which compares the
# ModelingToolkit modeling of every problem with the OptimalControl one, as
# `suite/comparison/test_modelings` does for JuMP (#441).
#
# What a ModelingToolkit problem writes, and how it is read:
# - the dynamics `D(xᵢ(t)) ~ fᵢ`, one equation per state;
# - the costs (minimized, summed): integrals `Integral(t in (t0, tf))(ℓ)` and terms in the state at
#   `t0` or `tf`; the times are those of the parameters of the problem;
# - the initial state, as the default value of a state that is not tunable;
# - the constraints `a ~ b`, `a ≲ b`, `a ≳ b`: a constraint in `t` is a path constraint, or a
#   bound when it constrains one state or one control; one in the state at `t0` or `tf` is a
#   boundary constraint;
# - the bounds of the metadata (`[bounds = (l, u)]`).
module ContinuousModelingToolkit

using CTModels: CTModels
using ModelingToolkit: ModelingToolkit as MTK

const SU = MTK.SymbolicUtils
const SYM = MTK.Symbolics

unwrap(e) = SYM.unwrap(e)

"""The value of `e` when it is a number, `nothing` otherwise."""
function constant(e)
    v = SU.unwrap_const(unwrap(e))
    return v isa Real ? Float64(v) : nothing
end

"""Name of a state or a control, without its argument: `x₁` for `x₁(t)`."""
name(s) = replace(string(s), r"\(.*\)$" => "")

"""Whether `e` depends on the independent variable `iv`."""
function depends_on_time(e, iv)
    e = unwrap(e)
    isequal(e, iv) && return true
    SU.iscall(e) || return false
    return any(a -> depends_on_time(a, iv), SU.arguments(e))
end

"""
The value of the expression `e` at the point `p`: `p.t`, `p.x`, `p.u` for a signal at the
time `t`, `p.x0`, `p.xf` for the state at `t0` or `tf`. `sig` holds the independent variable,
the states and the controls, and the times.
"""
function evaluate(e, sig, p)
    e = unwrap(e)
    isequal(e, sig.iv) && return p.t
    if SU.iscall(e)
        op, args = SU.operation(e), SU.arguments(e)
        for (signals, now, at_t0, at_tf) in
            ((sig.states, p.x, p.x0, p.xf), (sig.controls, p.u, nothing, nothing))
            i = findfirst(s -> isequal(op, SU.operation(s)), signals)
            isnothing(i) && continue
            arg = unwrap(only(args))
            isequal(arg, sig.iv) && return now[i]
            τ = constant(arg)
            !isnothing(at_t0) && τ == sig.t0 && return at_t0[i]
            !isnothing(at_tf) && τ == sig.tf && return at_tf[i]
            throw(ArgumentError("cannot evaluate $e: a signal at $τ, not at t0 or tf"))
        end
        return op(map(a -> evaluate(a, sig, p), args)...)
    end
    v = constant(e)
    isnothing(v) && throw(ArgumentError("cannot evaluate $e"))
    return v
end

point(; t=nothing, x=nothing, u=nothing, x0=nothing, xf=nothing) = (; t, x, u, x0, xf)

"""`h` as `(h′, c)` with `h = h′ + c` and `c` the constant term of a sum."""
function split_constant(h)
    h = unwrap(h)
    (SU.iscall(h) && SU.operation(h) === (+)) || return h, 0.0
    args = SU.arguments(h)
    values = map(constant, args)
    rest = [a for (a, c) in zip(args, values) if isnothing(c)]
    c = sum(c for c in values if !isnothing(c); init=0.0)
    return (length(rest) == 1 ? only(rest) : +(rest...)), c
end

"""
A constraint as `(lower, h, upper)`: `lower ≤ h ≤ upper`, the bounds being numbers. A
constraint between an expression and a number keeps the expression, as the other modelings
write it (`x(tf) == x_tf`).
"""
function bounded(c)
    if c isa SYM.Equation
        l, r = c.lhs, c.rhs
        cr, cl = constant(r), constant(l)
        !isnothing(cr) && return (cr, unwrap(l), cr)
        !isnothing(cl) && return (cl, unwrap(r), cl)
        h, k = split_constant(l - r)
        return (-k, h, -k)
    elseif c isa SYM.Inequality
        # l ≤ r
        l, r = c.relational_op === SYM.leq ? (c.lhs, c.rhs) : (c.rhs, c.lhs)
        cr, cl = constant(r), constant(l)
        !isnothing(cr) && return (-Inf, unwrap(l), cr)
        !isnothing(cl) && return (cl, unwrap(r), Inf)
        h, k = split_constant(l - r)
        return (-Inf, h, -k)
    end
    return throw(ArgumentError("unsupported constraint $c"))
end

"""
The rows `(lower, h, upper)` with the one-sided rows on the same expression `h` merged into one
two-sided row: `l ≤ h` and `h ≤ u` are written `l ≤ h ≤ u` by OptimalControl.
"""
function merged(rows)
    out = Tuple{Float64,Any,Float64}[]
    for (lo, h, hi) in rows
        i = findfirst(r -> isequal(r[2], h), out)
        if isnothing(i)
            push!(out, (lo, h, hi))
        else
            out[i] = (max(out[i][1], lo), h, min(out[i][3], hi))
        end
    end
    return out
end

"""Initial value of the state `x` when it is fixed (not tunable), `nothing` otherwise."""
function initial_value(sys, x)
    MTK.istunable(x, false) && return nothing
    v = nothing
    if isdefined(MTK, :initial_conditions)
        ics = MTK.initial_conditions(sys)
        v = get(ics, unwrap(x), nothing)
    end
    isnothing(v) && (v = SYM.getdefaultval(x, nothing))
    return isnothing(v) ? nothing : constant(v)
end

"""`syms` in the order of `names`; an error when a name is missing."""
function ordered(syms, names)
    return map(names) do n
        i = findfirst(s -> name(s) == n, syms)
        isnothing(i) && throw(ArgumentError("no component $n among $(name.(syms))"))
        return syms[i]
    end
end

"""
The data of the ModelingToolkit `System` `sys` in the fields of the JuMP `ProblemData`, read by
`Continuous.compare`; the states and the controls in the order of `state_names` and
`control_names` (those of the OptimalControl model), the times `(t0, tf)` being `tspan`: a
`System` does not say them when its cost has no integral.
"""
function problem_data(sys, state_names, control_names, tspan)
    CTModels.Models.variable_dimension(sys) == 0 ||
        throw(ArgumentError("a problem with a variable is not supported yet"))
    iv = unwrap(MTK.get_iv(sys))
    unknowns = MTK.unknowns(sys)
    xs = ordered(filter(!MTK.isinput, unknowns), state_names)
    us = ordered(filter(MTK.isinput, unknowns), control_names)
    n, m = length(xs), length(us)
    t0, tf = Float64.(tspan)

    # costs: integrals over (t0, tf) and terms in the state at t0 or tf
    integrands, terms = Any[], Any[]
    for c in MTK.get_costs(sys)
        c = unwrap(c)
        if SU.iscall(c) && SU.operation(c) isa SYM.Integral
            d = SU.operation(c).domain.domain
            (constant(d.left), constant(d.right)) == (t0, tf) ||
                throw(ArgumentError("an integral over another interval than (t0, tf)"))
            push!(integrands, only(SU.arguments(c)))
        else
            push!(terms, c)
        end
    end
    sig = (; iv, states=unwrap.(xs), controls=unwrap.(us), t0, tf)

    # dynamics, in the order of the states
    rhs = Vector{Any}(nothing, n)
    for eq in MTK.equations(sys)
        lhs = unwrap(eq.lhs)
        (SU.iscall(lhs) && SU.operation(lhs) isa SYM.Differential) ||
            throw(ArgumentError("not a dynamics: $eq"))
        i = findfirst(s -> isequal(s, unwrap(only(SU.arguments(lhs)))), sig.states)
        isnothing(i) && throw(ArgumentError("not the derivative of a state: $eq"))
        rhs[i] = eq.rhs
    end
    any(isnothing, rhs) && throw(ArgumentError("a state has no dynamics"))

    # bounds of the metadata, then the constraints
    function metadata_bounds(s)
        l, u = MTK.getbounds(s)
        return something(constant(l), -Inf), something(constant(u), Inf)
    end
    lower_x, upper_x = first.(metadata_bounds.(xs)), last.(metadata_bounds.(xs))
    lower_u, upper_u = first.(metadata_bounds.(us)), last.(metadata_bounds.(us))
    path_rows, boundary_rows = Tuple{Float64,Any,Float64}[], Tuple{Float64,Any,Float64}[]
    for c in MTK.get_constraints(sys)
        lo, h, hi = bounded(c)
        if depends_on_time(h, iv)
            i = findfirst(s -> isequal(s, h), sig.states)
            k = findfirst(s -> isequal(s, h), sig.controls)
            if !isnothing(i)
                lower_x[i], upper_x[i] = max(lower_x[i], lo), min(upper_x[i], hi)
            elseif !isnothing(k)
                lower_u[k], upper_u[k] = max(lower_u[k], lo), min(upper_u[k], hi)
            else
                push!(path_rows, (lo, h, hi))
            end
        else
            push!(boundary_rows, (lo, h, hi))
        end
    end
    path = Tuple{Any,Function,Any}[
        (lo, (x, u, v, t) -> evaluate(h, sig, point(; t, x, u)), hi) for
        (lo, h, hi) in merged(path_rows)
    ]
    boundary = Tuple{Any,Function,Any}[
        (lo, (x0, xf, v) -> evaluate(h, sig, point(; x0, xf)), hi) for
        (lo, h, hi) in merged(boundary_rows)
    ]
    for (i, x) in enumerate(xs)
        a = initial_value(sys, x)
        isnothing(a) || push!(boundary, (a, (x0, xf, v) -> x0[i], a))
    end

    component(s, l, u) = (; name=name(s), lower=l, upper=u)
    lagrange = if isempty(integrands)
        nothing
    else
        (x, u, v, t) -> sum(evaluate(ℓ, sig, point(; t, x, u)) for ℓ in integrands)
    end
    return (;
        state=[component(xs[i], lower_x[i], upper_x[i]) for i in 1:n],
        control=[component(us[k], lower_u[k], upper_u[k]) for k in 1:m],
        variable=NamedTuple[],
        times=v -> (t0, tf),
        dynamics=(x, u, v, t) -> [evaluate(f, sig, point(; t, x, u)) for f in rhs],
        lagrange=lagrange,
        cost=(I, x0, xf, v) ->
            I + sum(evaluate(g, sig, point(; x0, xf)) for g in terms; init=0.0),
        path=path,
        boundary=boundary,
    )
end

end # module ContinuousModelingToolkit
