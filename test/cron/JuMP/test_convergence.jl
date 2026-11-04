module TestConvergenceJuMP

using Test: Test
using JuMP: JuMP
using Ipopt: Ipopt
using OptimalControlProblems: OptimalControlProblems

const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

# Default grid of each problem, two solves in a row, as the former test_JuMP..

function test_convergence()
    options = Main.TestSolverOptions
    Test.@testset "JuMP convergence (default grid)" verbose = VERBOSE showtiming = SHOWTIMING begin
        for problem in OptimalControlProblems.problems()
            Test.@testset "$problem" begin
                try
                    build = getfield(OptimalControlProblems, problem)
                    model = build(OptimalControlProblems.JuMPBackend())
                    JuMP.set_optimizer(model, Ipopt.Optimizer)
                    JuMP.set_silent(model)
                    JuMP.set_optimizer_attribute(model, "tol", options.TOL)
                    JuMP.set_optimizer_attribute(model, "max_iter", options.MAX_ITER)
                    JuMP.set_optimizer_attribute(model, "mu_strategy", options.MU_STRATEGY)
                    JuMP.set_optimizer_attribute(model, "linear_solver", "mumps")
                    JuMP.set_optimizer_attribute(
                        model, "max_wall_time", options.MAX_WALL_TIME
                    )
                    JuMP.set_optimizer_attribute(model, "sb", options.SB)
                    JuMP.optimize!(model)
                    JuMP.optimize!(model) # second solve, warm
                    Main.TestHelpers.@broken_if_false JuMP.termination_status(model) ==
                        JuMP.MOI.LOCALLY_SOLVED
                catch err
                    Main.TestHelpers.broken_if_domain_error(err)
                end
            end
        end
    end
end

end # module TestConvergenceJuMP

test_convergence() = TestConvergenceJuMP.test_convergence()
