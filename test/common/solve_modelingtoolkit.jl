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
Extract the time horizon `(t0, tf)` from the integral cost domain in `sys`.
"""
function _extract_tspan(sys::MTK.AbstractSystem)
    costs = MTK.get_costs(sys)
    isempty(costs) && throw(ArgumentError("system has no cost terms"))
    cost = costs[1]
    op = MTK.SymbolicUtils.operation(cost)
    if hasproperty(op, :domain) && hasproperty(op.domain, :domain)
        d = op.domain.domain
        return (Float64(d.left), Float64(d.right))
    else
        throw(ArgumentError("cannot extract time horizon from cost $cost"))
    end
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
Prepare an `Optimization.OptimizationProblem` from a ModelingToolkit problem `pb` using Corleone.
"""
function prepare(
    pb;
    grid_size::Int=Main.TestConfigs.SMOKE_GRID,
    shooting::Symbol=:single,
    solver::Symbol=:ipopt,
)
    shooting === :single || throw(ArgumentError("unsupported shooting method $shooting"))
    sys = OCP.model(pb)
    iv = MTK.independent_variable(sys)
    t0, tf = _extract_tspan(sys)
    grid = range(t0, tf, length=grid_size)

    # Discretize path constraints over the grid
    sys_disc = _discretize_path_constraints(sys, grid)

    # Controls and initial values
    ctrl_syms = filter(MTK.isinput, MTK.unknowns(sys))
    inits = [u => 0.0 for u in ctrl_syms]
    # Control switches: interior switches preceded by tspan[1]
    switches = [u => collect(grid[1:end-1]) for u in ctrl_syms]

    opt_prob = Corleone.CorleoneDynamicOptProblem(
        sys_disc,
        inits,
        switches...;
        algorithm=Tsit5(),
        tspan=(t0, tf),
    )

    return Optimization.OptimizationProblem(opt_prob, Optimization.AutoForwardDiff())
end

"""
Solve the `Problem` `pb` with `config`.
"""
function solve(
    pb,
    config=(shooting=:single, solver=:ipopt);
    grid_size::Int=Main.TestConfigs.SMOKE_GRID,
    max_iter::Int=Main.TestConfigs.MAX_ITER,
    options::NamedTuple=(;),
)
    prob = prepare(
        pb;
        grid_size=grid_size,
        shooting=get(config, :shooting, :single),
        solver=get(config, :solver, :ipopt),
    )
    opts = solver_options(get(config, :solver, :ipopt); max_iter=max_iter, options=options)
    res = Optimization.solve(prob, Ipopt.Optimizer(); opts...)
    return res
end

"""
How a solve with `config` and `options` was configured.
"""
function settings(config; max_iter::Int=Main.TestConfigs.MAX_ITER, options::NamedTuple=(;))
    return (
        modeler=:corleone,
        shooting=get(config, :shooting, :single),
        solver=config.solver,
        options=solver_options(config.solver; max_iter=max_iter, options=options),
    )
end

"""
Whether the optimization succeeded.
"""
function succeeded(res)
    return Optimization.SciMLBase.successful_retcode(res) ||
           res.retcode == Optimization.ReturnCode.Success
end

"""
What `TestStatus.result!` records of a solution.
"""
function outcome(res)
    iters = hasproperty(res, :stats) && hasproperty(res.stats, :iterations) ? res.stats.iterations : 0
    return (
        converged=succeeded(res),
        reason=res.retcode,
        objective=res.objective,
        iterations=iters,
        violation=0.0,
    )
end

"""
Reconstruct the `Corleone.Trajectory` from an optimization solution `res` and problem `pb`.
"""
function dynamic_trajectory(
    res,
    pb;
    grid_size::Int=Main.TestConfigs.SMOKE_GRID,
    shooting::Symbol=:single,
)
    shooting === :single || throw(ArgumentError("unsupported shooting method $shooting"))
    sys = OCP.model(pb)
    iv = MTK.independent_variable(sys)
    t0, tf = _extract_tspan(sys)
    grid = range(t0, tf, length=grid_size)

    # Discretize path constraints over the grid
    sys_disc = _discretize_path_constraints(sys, grid)

    # Controls and initial values
    ctrl_syms = filter(MTK.isinput, MTK.unknowns(sys))
    inits = [u => 0.0 for u in ctrl_syms]
    switches = [u => collect(grid[1:(end - 1)]) for u in ctrl_syms]

    opt_prob = Corleone.CorleoneDynamicOptProblem(
        sys_disc,
        inits,
        switches...;
        algorithm=Tsit5(),
        tspan=(t0, tf),
    )

    _, reconstruct = Corleone.pack(Val(:ComponentArrays), opt_prob.ps)
    current = reconstruct(res.u)
    traj, _ = opt_prob.layer(
        (opt_prob.layer.problem, opt_prob.evaluation_grid), current, opt_prob.st
    )
    return traj
end

"""
The trajectory of a solution `res` for the plots of the documentation (#314).
Matches the schema of `TestStatus.trajectory` with empty costates as Corleone is a
direct shooting solver without explicit costates.
"""
function trajectory(
    res,
    pb;
    grid_size::Int=Main.TestConfigs.SMOKE_GRID,
    shooting::Symbol=:single,
)
    (hasproperty(res, :u) && !isnothing(res.u)) || return nothing
    try
        traj = dynamic_trajectory(res, pb; grid_size=grid_size, shooting=shooting)
        sys = OCP.model(pb)
        t = collect(traj.t)
        n_points = length(t)

        # Points kept on the time grid: at most 501 points like TestStatus.TRAJECTORY_POINTS
        max_points = 501
        keep = if n_points <= max_points
            eachindex(t)
        else
            unique(round.(Int, range(1, n_points; length=max_points)))
        end
        t_sub = t[keep]
        r(v) = round(Float64(v); sigdigits=6)

        xs = CTModels.Models.state_components(sys)
        us = CTModels.Models.control_components(sys)
        vs = CTModels.Models.variable_components(sys)

        state_dict = Dict{String,Vector{Float64}}()
        for name in xs
            sym = Symbol(name)
            vals = traj[sym]
            state_dict[name] = [r(vals[i]) for i in keep]
        end

        control_dict = Dict{String,Vector{Float64}}()
        for name in us
            sym = Symbol(name)
            vals = traj[sym]
            # Control is piecewise-constant on intervals (length n_points - 1).
            # Extend to n_points so it aligns with the time grid.
            full_vals = [vals[min(i, length(vals))] for i in 1:n_points]
            control_dict[name] = [r(full_vals[i]) for i in keep]
        end

        return Dict{String,Any}(
            "time" => r.(t_sub),
            "state" => state_dict,
            "control" => control_dict,
            "costate" => Dict{String,Vector{Float64}}(),
            "variable" => Float64[],
            "names" => Dict("state" => xs, "control" => us, "variable" => vs),
        )
    catch err
        @warn "No trajectory for this ModelingToolkit solve" exception = err
        return nothing
    end
end

end # module SolveModelingToolkit
