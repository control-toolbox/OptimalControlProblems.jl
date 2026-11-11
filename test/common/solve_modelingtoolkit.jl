# Solve a ModelingToolkit optimal control problem via Corleone.
module SolveModelingToolkit

using ComponentArrays: ComponentArrays
using Corleone: Corleone
using CTModels: CTModels
using Ipopt: Ipopt
using ModelingToolkit: ModelingToolkit as MTK
using Optimization: Optimization
using OptimizationMOI: OptimizationMOI
using OrdinaryDiffEqTsit5: Tsit5
using OptimalControlProblems: OptimalControlProblems as OCP

"""
Whether the final time of the `System` `sys` is free: the problem has a variable `tf`, a
parameter to optimize. Corleone integrates on a fixed horizon, so such a problem is written
in the reduced time `s = (t - t0) / (tf - t0)` on `[0, 1]`, and its derivatives are multiplied
by the duration `tf - t0`.
"""
free_final_time(sys) = "tf" in CTModels.Models.variable_components(sys)

"""
The physical times `(t0, tf)` of the problem `pb`, from its parameters (a `System` does not say
them when its cost has no integral, a Mayer cost); `tf` is `nothing` when the final time is
free, the variable `tf` being then optimized.
"""
function times(pb)
    params = OCP.metadata(pb)[:parameters]
    t0 = Float64(params.t0)
    free_final_time(OCP.model(pb)) && return (t0, nothing)
    return (t0, Float64(params.tf))
end

"""
The time horizon the `System` of `pb` is integrated on: `(t0, tf)`, or `(0, 1)` when the final
time is free (reduced time, see `free_final_time`).
"""
function tspan(pb)
    t0, tf = times(pb)
    return isnothing(tf) ? (0.0, 1.0) : (t0, tf)
end

"""
Recursively replace the independent variable `iv` by a numerical value `τ` in an expression.
"""
function _sub_iv(expr, iv, τ)
    if isequal(expr, iv)
        return τ
    elseif expr isa MTK.Symbolics.Equation
        return MTK.Symbolics.Equation(_sub_iv(expr.lhs, iv, τ), _sub_iv(expr.rhs, iv, τ))
    elseif hasproperty(expr, :relational_op) && hasproperty(expr, :lhs) && hasproperty(expr, :rhs)
        return typeof(expr)(_sub_iv(expr.lhs, iv, τ), _sub_iv(expr.rhs, iv, τ), expr.relational_op)
    elseif MTK.SymbolicUtils.iscall(expr)
        op = MTK.SymbolicUtils.operation(expr)
        args = MTK.SymbolicUtils.arguments(expr)
        new_args = [_sub_iv(a, iv, τ) for a in args]
        return op(new_args...)
    else
        return expr
    end
end

"""
Recursively check whether an expression contains the independent variable `iv`.
"""
function _contains_iv(expr, iv)
    isequal(expr, iv) && return true
    if hasproperty(expr, :lhs) && hasproperty(expr, :rhs)
        return _contains_iv(expr.lhs, iv) || _contains_iv(expr.rhs, iv)
    elseif MTK.SymbolicUtils.iscall(expr)
        return any(a -> _contains_iv(a, iv), MTK.SymbolicUtils.arguments(expr))
    end
    return false
end

"""
Check whether a constraint depends on the independent variable `iv` as a free argument.
"""
function _is_path_constraint(c, iv)
    return _contains_iv(c, iv)
end

"""
Discretize path constraints over a temporal grid so that Corleone can evaluate them.
"""
function _discretize_path_constraints(sys::MTK.AbstractSystem, grid::AbstractVector)
    iv = MTK.independent_variable(sys)
    cons = MTK.get_constraints(sys)
    new_cons = Any[]
    for c in cons
        if _is_path_constraint(c, iv)
            for τ in grid
                push!(new_cons, _sub_iv(c, iv, τ))
            end
        else
            push!(new_cons, c)
        end
    end
    return MTK.complete(
        MTK.System(
            MTK.equations(sys),
            iv;
            name=MTK.nameof(sys),
            constraints=new_cons,
            costs=MTK.get_costs(sys),
        )
    )
end

"""
The attributes of the optimizer for a solve: the defaults of `Main.TestConfigs`, replaced by
`options`.
"""
function solver_options(
    solver::Symbol=:ipopt; max_iter::Int=Main.TestConfigs.MAX_ITER, options::NamedTuple=(;)
)
    solver === :ipopt || throw(ArgumentError("unsupported ModelingToolkit solver $solver"))
    o = Main.TestConfigs
    defaults = (
        tol=o.TOL,
        max_iter=max_iter,
        mu_strategy=o.MU_STRATEGY,
        linear_solver="mumps",
        max_wall_time=o.MAX_WALL_TIME,
        sb=o.SB,
        print_level=0,
    )
    return merge(defaults, options)
end

"""
The start value of each of the `m` controls, from the initial guess `init` of the problem
(#377): a number, a vector, or a function of the time taken at `t0`; `0.0` without a control
guess. Corleone starts every interval of a control from one value.
"""
function control_start(init, t0, m::Int)
    return _start(init, :control, t0, m)
end

"""
The start value of each of the `n` states, from the initial guess `init` of the problem, as
`control_start`; used for the states whose initial value is free, `0.0` without a state guess.
"""
function state_start(init, t0, n::Int)
    return _start(init, :state, t0, n)
end

"""The parameters of the `System` `sys` that are variables of the problem, as `variable_components`."""
function variable_symbols(sys)
    return filter(p -> MTK.istunable(p, true) && !MTK.isinput(p), MTK.parameters(sys))
end

"""The start value of each of the `n` variables, from the initial guess `init` of the problem."""
variable_start(init, n::Int) = _start(init, :variable, 0.0, n)

function _start(init, key::Symbol, t0, n::Int)
    g = haskey(init, key) ? init[key] : 0.0
    g isa Function && (g = g(t0))
    v = g isa Number ? fill(Float64(g), n) : Float64.(vec(collect(g)))
    length(v) == n || throw(DimensionMismatch("$key guess of length $(length(v)), $n $(key)s"))
    return v
end

"""
Prepare the solve of the ModelingToolkit problem `pb` with Corleone, built once: the path
constraints imposed at the `grid_size` nodes of the grid, the controls piecewise constant
between them and started from the initial guess of the problem, as the states whose initial
value is free (`tunable = true`). Returns the
`Optimization.OptimizationProblem` (`problem`) and the `Corleone.CorleoneDynamicOptProblem`
it was built from (`dynamic`), from which the trajectory of a solution is recovered.
"""
function prepare(
    pb;
    grid_size::Int=Main.TestConfigs.SMOKE_GRID,
    shooting::Symbol=:single,
    solver::Symbol=:ipopt,
)
    shooting === :single || throw(ArgumentError("unsupported shooting method $shooting"))
    sys = OCP.model(pb)
    t0, tf = tspan(pb)
    grid = range(t0, tf, length=grid_size)

    # Discretize path constraints over the grid
    sys_disc = _discretize_path_constraints(sys, grid)

    # Controls: started from the initial guess, switching at the nodes of the grid
    ctrl_syms = filter(MTK.isinput, MTK.unknowns(sys))
    init = OCP.initial_guess(pb)
    start = control_start(init, t0, length(ctrl_syms))
    inits = [u => start[i] for (i, u) in enumerate(ctrl_syms)]

    # States whose initial value is free: started from the initial guess. The order of the
    # states is the one of `state_components`, that of the OptimalControl model.
    states = filter(!MTK.isinput, MTK.unknowns(sys))
    start_x = state_start(init, t0, length(states))
    append!(inits, [x => start_x[i] for (i, x) in enumerate(states) if MTK.istunable(x, false)])

    # The variables (the final time `tf`) are parameters to optimize, started from the guess.
    vars = variable_symbols(sys)
    start_v = variable_start(init, length(vars))
    append!(inits, [v => start_v[i] for (i, v) in enumerate(vars)])
    switches = [u => collect(grid[1:(end - 1)]) for u in ctrl_syms]

    dynamic = Corleone.CorleoneDynamicOptProblem(
        sys_disc,
        inits,
        switches...;
        algorithm=Tsit5(),
        tspan=(t0, tf),
    )
    problem = Optimization.OptimizationProblem(dynamic, Optimization.AutoForwardDiff())
    return (; problem, dynamic)
end

"""
Solve the `Problem` `pb` with `config`: a named tuple with the Optimization solution
(`result`) and the problems it solves (`problem`, `dynamic`, see `prepare`).
"""
function solve(
    pb,
    config=(shooting=:single, solver=:ipopt);
    grid_size::Int=Main.TestConfigs.SMOKE_GRID,
    max_iter::Int=Main.TestConfigs.MAX_ITER,
    options::NamedTuple=(;),
)
    prepared = prepare(
        pb;
        grid_size=grid_size,
        shooting=get(config, :shooting, :single),
        solver=get(config, :solver, :ipopt),
    )
    opts = solver_options(get(config, :solver, :ipopt); max_iter=max_iter, options=options)
    result = Optimization.solve(prepared.problem, Ipopt.Optimizer(); opts...)
    return (; result, prepared...)
end

"""
The values of the control `name` (`"u"`) the problem `dynamic` of a `prepare` starts from:
one per interval.
"""
function control_values(dynamic, name)
    return dynamic.ps.parameter.parameters[Symbol(name, "(t)")].parameter.ps
end

"""
The values of the variable `name` (`"tf"`) the problem `dynamic` of a `prepare` starts from.
"""
function variable_values(dynamic, name)
    return dynamic.ps.parameter.parameters[Symbol(name)].parameter.ps
end

"""The objective of a solution `sol` of `solve`."""
objective(sol) = sol.result.objective

"""
How a solve with `config` and `options` was configured, as `TestStatus.result!` records it.
The shooting is in the scheme, see `scheme`.
"""
function settings(config; max_iter::Int=Main.TestConfigs.MAX_ITER, options::NamedTuple=(;))
    return (
        modeler=:corleone,
        solver=config.solver,
        options=solver_options(config.solver; max_iter=max_iter, options=options),
    )
end

"""The discretization of a solve with `config`, for the report: `:shooting_single`."""
scheme(config) = Symbol(:shooting_, get(config, :shooting, :single))

"""
Whether the optimization succeeded.
"""
function succeeded(sol)
    res = sol.result
    return Optimization.SciMLBase.successful_retcode(res) ||
           res.retcode == Optimization.ReturnCode.Success
end

"""
The largest violation of the constraints of the NLP solved (the path constraints at the nodes
of the grid, the boundary constraints) and of the bounds of its unknowns, at the solution.
"""
function violation(sol)
    prob, u = sol.problem, sol.result.u
    v = 0.0
    if !isnothing(prob.lb)
        v = max(v, maximum(max.(prob.lb .- u, u .- prob.ub, 0.0); init=0.0))
    end
    if !isnothing(prob.f.cons) && !isnothing(prob.lcons)
        c = zeros(length(prob.lcons))
        prob.f.cons(c, u, prob.p)
        v = max(v, maximum(max.(prob.lcons .- c, c .- prob.ucons, 0.0); init=0.0))
    end
    return v
end

"""
What `TestStatus.result!` records of a solution.
"""
function outcome(sol)
    res = sol.result
    iters = if hasproperty(res, :stats) && hasproperty(res.stats, :iterations)
        res.stats.iterations
    else
        0
    end
    return (
        converged=succeeded(sol),
        reason=res.retcode,
        objective=res.objective,
        iterations=iters,
        violation=violation(sol),
    )
end

"""
The `Corleone.Trajectory` of a solution `sol` of `solve`, from the problem it solved: the
states at the nodes of the grid and the controls.
"""
function dynamic_trajectory(sol)
    dynamic = sol.dynamic
    traj, _ = dynamic.layer(
        (dynamic.layer.problem, dynamic.evaluation_grid), solved_parameters(sol), dynamic.st
    )
    return traj
end

"""The parameters of the problem solved by `sol`, at the solution."""
function solved_parameters(sol)
    _, reconstruct = Corleone.pack(Val(:ComponentArrays), sol.dynamic.ps)
    return reconstruct(sol.result.u)
end

"""
The trajectory of a solution `sol` of `solve` for the plots of the documentation (#314), in
the schema of `TestStatus.trajectory`. The costates are empty: Corleone, a direct shooting
method, computes none.
"""
function trajectory(sol, pb)
    (hasproperty(sol.result, :u) && !isnothing(sol.result.u)) || return nothing
    try
        return _trajectory(sol, pb)
    catch err
        @warn "No trajectory for this ModelingToolkit solve" exception = err
        return nothing
    end
end

function _trajectory(sol, pb)
    traj = dynamic_trajectory(sol)
    sys = OCP.model(pb)
    t = collect(traj.t)
    n_points = length(t)

    # The variables at the solution; with a free final time, the time is reduced to [0, 1]
    # and goes back to the physical time t0 + (tf - t0) s.
    xs = CTModels.Models.state_components(sys)
    us = CTModels.Models.control_components(sys)
    vs = CTModels.Models.variable_components(sys)
    solved = [only(solved_parameters(sol).parameter.parameters[Symbol(v)].parameter.ps) for v in vs]
    if free_final_time(sys)
        t0, _ = times(pb)
        t = t0 .+ (only(solved[findall(==("tf"), vs)]) - t0) .* t
    end

    # At most TestStatus.TRAJECTORY_POINTS points, as for the other modelings
    max_points = Main.TestStatus.TRAJECTORY_POINTS
    keep = if n_points <= max_points
        eachindex(t)
    else
        unique(round.(Int, range(1, n_points; length=max_points)))
    end
    r(v) = round(Float64(v); sigdigits=6)

    state = Dict{String,Vector{Float64}}()
    for name in xs
        vals = traj[Symbol(name)]
        length(vals) == n_points ||
            throw(DimensionMismatch("$(length(vals)) values of $name, $n_points times"))
        state[name] = [r(vals[i]) for i in keep]
    end

    # A control is constant on each interval: one value per interval (n_points - 1), the last
    # one extended to the final time, or one value per time.
    control = Dict{String,Vector{Float64}}()
    for name in us
        vals = traj[Symbol(name)]
        length(vals) in (n_points - 1, n_points) ||
            throw(DimensionMismatch("$(length(vals)) values of $name, $n_points times"))
        full = [vals[min(i, length(vals))] for i in 1:n_points]
        control[name] = [r(full[i]) for i in keep]
    end

    return Dict{String,Any}(
        "time" => r.(t[keep]),
        "state" => state,
        "control" => control,
        "costate" => Dict{String,Vector{Float64}}(),
        "variable" => r.(solved),
        "names" => Dict("state" => xs, "control" => us, "variable" => vs),
    )
end

end # module SolveModelingToolkit
