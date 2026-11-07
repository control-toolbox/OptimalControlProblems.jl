# Solve a JuMP model with the options of `Main.TestConfigs` (environments JuMP, comparison).
module SolveJuMP

using JuMP: JuMP
using Ipopt: Ipopt

"""
The attributes of the optimizer for a solve: the defaults of `Main.TestConfigs`, replaced by
`options` (robustness, #378). The results of the solves record them (#409).
"""
function solver_options(
    solver::Symbol=:ipopt; max_iter::Int=Main.TestConfigs.MAX_ITER, options::NamedTuple=(;)
)
    solver === :ipopt || throw(ArgumentError("unsupported JuMP solver $solver"))
    o = Main.TestConfigs
    defaults = (
        tol=o.TOL,
        max_iter=max_iter,
        mu_strategy=o.MU_STRATEGY,
        linear_solver="mumps",
        max_wall_time=o.MAX_WALL_TIME,
        sb=o.SB,
    )
    return merge(defaults, options)
end

function solve!(
    model;
    solver::Symbol=:ipopt,
    max_iter::Int=Main.TestConfigs.MAX_ITER,
    options::NamedTuple=(;),
)
    all = solver_options(solver; max_iter=max_iter, options=options)
    JuMP.set_optimizer(model, Ipopt.Optimizer)
    JuMP.set_silent(model)
    for (name, value) in pairs(all)
        JuMP.set_optimizer_attribute(model, string(name), value)
    end
    JuMP.optimize!(model)
    return model
end

"""
How a solve with `config` and `options` was configured, as `TestStatus.result!` records it
(#409): no modeler (the model is JuMP's), the solver and every attribute of the optimizer.
"""
function settings(config; max_iter::Int=Main.TestConfigs.MAX_ITER, options::NamedTuple=(;))
    return (
        modeler=nothing,
        solver=config.solver,
        options=solver_options(config.solver; max_iter=max_iter, options=options),
    )
end

succeeded(model) = JuMP.termination_status(model) == JuMP.MOI.LOCALLY_SOLVED

"""
Largest violation of the constraints at the solution (#409), `NaN` when there is none: what
`OptimalControl.constraints_violation` gives for the other modelings.
"""
function violation(model)
    JuMP.has_values(model) || return NaN
    report = JuMP.primal_feasibility_report(model)
    return isempty(report) ? 0.0 : Float64(maximum(values(report)))
end

"""What `TestStatus.result!` records of a solved model."""
function outcome(model)
    return (
        converged=succeeded(model),
        reason=JuMP.termination_status(model),
        objective=JuMP.has_values(model) ? JuMP.objective_value(model) : NaN,
        iterations=JuMP.barrier_iterations(model),
        violation=violation(model),
    )
end

end # module SolveJuMP
