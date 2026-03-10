# test_JuMP_optimality
function test_JuMP()
    for f in LIST_OF_PROBLEMS
        @testset "$(f)" verbose=VERBOSE begin
            try
                grid_size = metadata(f)[:grid_size]

                # do we keep or remove the problem from the list
                keep_problem = true

                #
                DEBUG && println("\n", "┌─ ", string(f), " (JuMP)")
                DEBUG && println("│")

                # Set up the model
                nlp = OptimalControlProblems.eval(f)(JuMPBackend(); grid_size=grid_size)
                set_optimizer(nlp, Ipopt.Optimizer)
                set_silent(nlp)
                set_optimizer_attribute(nlp, "tol", TOL)
                set_optimizer_attribute(nlp, "max_iter", MAX_ITER)
                set_optimizer_attribute(nlp, "mu_strategy", MU_STRATEGY)
                set_optimizer_attribute(nlp, "linear_solver", "mumps")
                set_optimizer_attribute(nlp, "max_wall_time", MAX_WALL_TIME)
                set_optimizer_attribute(nlp, "sb", SB)

                # check existence of required metadata
                nlp_keys = keys(object_dictionary(nlp))
                @test :time_grid ∈ nlp_keys
                @test :state_components ∈ nlp_keys
                @test :costate_components ∈ nlp_keys
                @test :control_components ∈ nlp_keys
                @test :variable_components ∈ nlp_keys

                # check if the keys from the components names exists
                components = [
                    :state_components,
                    :costate_components,
                    :control_components,
                    :variable_components,
                ]
                for c in components
                    if !(isnothing(nlp[c]))
                        for e in nlp[c]
                            @test Symbol(e) ∈ nlp_keys
                        end
                    end
                end

                # Solve the model
                print("  First solve:  ");
                @time optimize!(nlp)
                print("  Second solve: ");
                @time optimize!(nlp)

                # Infos
                DEBUG && println("│")
                DEBUG && print(
                    "│ termination_status: ",
                    termination_status(nlp),
                    ", objective: ",
                    objective_value(nlp),
                    ", iterations: ",
                    barrier_iterations(nlp),
                )

                # Test
                res = @my_test_broken termination_status(nlp) == MOI.LOCALLY_SOLVED
                keep_problem = keep_problem && res
                DEBUG && res && println(", \033[1;32mPass\033[0m")
                DEBUG && !res && println(", \033[1;31mFail\033[0m")
                DEBUG && println("│")
                DEBUG && println("└─")

                # do we keep or remove the problem from the list
                if !keep_problem
                    global LIST_OF_PROBLEMS_FINAL
                    LIST_OF_PROBLEMS_FINAL = setdiff(LIST_OF_PROBLEMS_FINAL, [f])
                end
            catch e
                handle_solver_error(e, f)
            end
        end
    end
end
