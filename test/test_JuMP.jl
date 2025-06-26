# test_JuMP_optimality
function test_JuMP()
    for f in list_of_problems
        @testset "$(f)" begin
            println("  $f:")
            # Set up the model
            model = OptimalControlProblems.eval(f)(JuMPBackend())
            set_optimizer(model, Ipopt.Optimizer)
            set_silent(model)
            set_optimizer_attribute(model, "tol", tol)
            set_optimizer_attribute(model, "constr_viol_tol", constr_viol_tol)
            set_optimizer_attribute(model, "max_iter", max_iter)
            set_optimizer_attribute(model, "mu_strategy", mu_strategy)
            set_optimizer_attribute(model, "linear_solver", "mumps")
            set_optimizer_attribute(model, "max_wall_time", max_wall_time)
            set_optimizer_attribute(model, "sb", sb)
            # Solve the model
            print("  First solve:  "); @time optimize!(model)
            print("  Second solve: "); @time optimize!(model)
            # Test that the solver found an optimal solution
            println("  termination_status = $(termination_status(model))\n")
            if f == :truck_trailer || f == :quadrotor
                @test (termination_status(model) == MOI.LOCALLY_INFEASIBLE) || (termination_status(model) == MOI.ITERATION_LIMIT)
            else
                @test termination_status(model) == MOI.LOCALLY_SOLVED
            end
        end
    end
end
