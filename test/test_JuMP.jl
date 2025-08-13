# test_JuMP_optimality
function test_JuMP()
    for f in LIST_OF_PROBLEMS
        @testset "$(f)" verbose=VERBOSE begin
            N = OptimalControlProblems.metadata[f][:N]

            # do we keep or remove the problem from the list
            keep_problem = true

            #
            DEBUG && println("\n", "┌─ ", string(f), " (JuMP)")
            DEBUG && println("│")

            # Set up the model
            model = OptimalControlProblems.eval(f)(JuMPBackend(); N=N)
            set_optimizer(model, Ipopt.Optimizer)
            set_silent(model)
            set_optimizer_attribute(model, "tol", TOL)
            set_optimizer_attribute(model, "max_iter", MAX_ITER)
            set_optimizer_attribute(model, "mu_strategy", MU_STRATEGY)
            set_optimizer_attribute(model, "linear_solver", "mumps")
            set_optimizer_attribute(model, "max_wall_time", MAX_WALL_TIME)
            set_optimizer_attribute(model, "sb", SB)

            # Solve the model
            DEBUG && println("├─  Solve")
            DEBUG && println("│")
            print("  First solve:  ");
            @time optimize!(model)
            print("  Second solve: ");
            @time optimize!(model)
            DEBUG && println("│")

            # Infos
            DEBUG && println("├─  Infos")
            DEBUG && println("│")
            DEBUG && println("│     termination_status: ", termination_status(model))
            DEBUG && println("│     objective: ", objective_value(model))
            DEBUG && println("│     iterations: ", barrier_iterations(model))
            DEBUG && println("│")

            # Test
            res = @my_test_broken termination_status(model) == MOI.LOCALLY_SOLVED
            keep_problem = keep_problem && (typeof(res) == Test.Pass)
            DEBUG &&
                (typeof(res) == Test.Pass) &&
                println("│     \033[1;32mTest Passed\033[0m")
            DEBUG &&
                (typeof(res) != Test.Pass) &&
                println("│     \033[1;31mTest Failed\033[0m")
            DEBUG && println("│")
            DEBUG && println("└─")

            # do we keep or remove the problem from the list
            if !keep_problem
                global LIST_OF_PROBLEMS_FINAL
                LIST_OF_PROBLEMS_FINAL = setdiff(LIST_OF_PROBLEMS_FINAL, [f])
            end
        end
    end
end
