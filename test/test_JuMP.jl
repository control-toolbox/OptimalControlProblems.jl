# test_JuMP_optimality
function test_JuMP()
    for f in LIST_OF_PROBLEMS
        @testset "$(f)" verbose=VERBOSE begin
            N = metadata(f)[:grid_size]

            # do we keep or remove the problem from the list
            keep_problem = true

            #
            DEBUG && println("\n", "┌─ ", string(f), " (JuMP)")
            DEBUG && println("│")

            # Set up the model
            nlp = OptimalControlProblems.eval(f)(JuMPBackend(); N=N)
            set_optimizer(nlp, Ipopt.Optimizer)
            set_silent(nlp)
            set_optimizer_attribute(nlp, "tol", TOL)
            set_optimizer_attribute(nlp, "max_iter", MAX_ITER)
            set_optimizer_attribute(nlp, "mu_strategy", MU_STRATEGY)
            set_optimizer_attribute(nlp, "linear_solver", "mumps")
            set_optimizer_attribute(nlp, "max_wall_time", MAX_WALL_TIME)
            set_optimizer_attribute(nlp, "sb", SB)

            # Solve the model
            DEBUG && println("├─  Solve")
            DEBUG && println("│")
            print("  First solve:  ");
            @time optimize!(nlp)
            print("  Second solve: ");
            @time optimize!(nlp)
            DEBUG && println("│")

            # Infos
            DEBUG && println("├─  Infos")
            DEBUG && println("│")
            DEBUG && println("│     termination_status: ", termination_status(nlp))
            DEBUG && println("│     objective: ", objective_value(nlp))
            DEBUG && println("│     iterations: ", barrier_iterations(nlp))
            DEBUG && println("│")

            # Test
            res = @my_test_broken termination_status(nlp) == MOI.LOCALLY_SOLVED
            keep_problem = keep_problem && res
            DEBUG &&  res && println("│     \033[1;32mTest Passed\033[0m")
            DEBUG && !res && println("│     \033[1;31mTest Failed\033[0m")
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
