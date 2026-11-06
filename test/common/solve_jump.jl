# Solve a JuMP model with the options of `Main.TestConfigs` (environments JuMP, comparison).
module SolveJuMP

using JuMP: JuMP
using Ipopt: Ipopt

function solve!(
    model; solver::Symbol=:ipopt, max_iter::Int=Main.TestConfigs.MAX_ITER, options::NamedTuple=(;)
)
    solver === :ipopt || throw(ArgumentError("unsupported JuMP solver $solver"))
    o = Main.TestConfigs
    JuMP.set_optimizer(model, Ipopt.Optimizer)
    JuMP.set_silent(model)
    JuMP.set_optimizer_attribute(model, "tol", o.TOL)
    JuMP.set_optimizer_attribute(model, "max_iter", max_iter)
    JuMP.set_optimizer_attribute(model, "mu_strategy", o.MU_STRATEGY)
    JuMP.set_optimizer_attribute(model, "linear_solver", "mumps")
    JuMP.set_optimizer_attribute(model, "max_wall_time", o.MAX_WALL_TIME)
    JuMP.set_optimizer_attribute(model, "sb", o.SB)
    # other options (robustness, #378) replace the defaults above
    for (name, value) in pairs(options)
        JuMP.set_optimizer_attribute(model, string(name), value)
    end
    JuMP.optimize!(model)
    return model
end

succeeded(model) = JuMP.termination_status(model) == JuMP.MOI.LOCALLY_SOLVED

"""What `TestStatus.result!` records of a solved model."""
function outcome(model)
    return (
        converged=succeeded(model),
        reason=JuMP.termination_status(model),
        objective=JuMP.has_values(model) ? JuMP.objective_value(model) : NaN,
        iterations=JuMP.barrier_iterations(model),
    )
end

end # module SolveJuMP
