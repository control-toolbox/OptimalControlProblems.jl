# The data of a problem, from which a scheme builds its JuMP model (#368). A problem file
# fills a `ProblemData` and calls a scheme; it writes no discretization itself.

"""
$(TYPEDEF)

A component of the state, the control or the variable: its name (also the name of the JuMP
container) and bounds. A bound is a number, or for the state and the control a vector over
the nodes `0:N`; `nothing` leaves it unset. The start values come from the initial guess of
the problem (#377), not from the components.
"""
struct Component
    name::String
    lower::Any
    upper::Any
end

function Component(name::AbstractString; lower=nothing, upper=nothing)
    return Component(String(name), lower, upper)
end

"""
$(TYPEDEF)

The data of an optimal control problem, written once for every scheme.

The functions receive, at one node, the vectors `x` (state), `u` (control), `v`
(variable) of JuMP variables and the time `t`; `x0`, `xf` are the state at the first and
last nodes.

- `times(v)`: the initial and final times, numbers or expressions of `v`.
- `dynamics(x, u, v, t)`: the vector `ẋ`.
- `lagrange(x, u, v, t)`: the integrand of the cost, or `nothing`.
- `cost(I, x0, xf, v)`: the objective, given the discretized integral `I` of the Lagrange
  cost (`0` without one).
- `path`: `(lower, f, upper)` with `f(x, u, v, t)`, imposed at every node.
- `boundary`: `(lower, f, upper)` with `f(x0, xf, v)`.
"""
Base.@kwdef struct ProblemData
    state::Vector{Component}
    control::Vector{Component}
    variable::Vector{Component} = Component[]
    costate::Vector{String}
    times::Function
    dynamics::Function
    lagrange::Union{Nothing,Function} = nothing
    cost::Function = (I, x0, xf, v) -> I
    path::Vector{Tuple{Any,Function,Any}} = Tuple{Any,Function,Any}[]
    boundary::Vector{Tuple{Any,Function,Any}} = Tuple{Any,Function,Any}[]
end
