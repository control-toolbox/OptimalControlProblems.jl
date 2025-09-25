# test_OptimalControl_optimality
function test_OptimalControl()
    options_ipopt = Dict(
        :print_level => 0,
        :tol => TOL,
        :mu_strategy => MU_STRATEGY,
        :sb => SB,
        :max_iter => MAX_ITER,
        :max_wall_time => MAX_WALL_TIME,
    )

    for f in LIST_OF_PROBLEMS
        @testset "$(f)" verbose=VERBOSE begin
            grid_size = metadata(f)[:grid_size]

            # do we keep or remove the problem from the list
            keep_problem = true

            #
            DEBUG && println("\n", "┌─ ", string(f), " (OptimalControl)")
            DEBUG && println("│")

            # Set up the model
            docp = OptimalControlProblems.eval(f)(OptimalControlBackend(); grid_size=grid_size)
            nlp = nlp_model(docp)

            # Solve the model
            print("  First solve:  ");
            @time sol = NLPModelsIpopt.ipopt(nlp; options_ipopt...)
            print("  Second solve: ");
            @time sol = NLPModelsIpopt.ipopt(nlp; options_ipopt...)

            # Infos
            DEBUG && println("│")
            DEBUG && print(  "│ sol.status: ", sol.status, ", objective: ", sol.objective, ", iterations: ", sol.iter)
            
            # Test
            res = @my_test_broken (sol.status == :first_order || sol.status == :acceptable)
            keep_problem = keep_problem && res
            DEBUG &&  res && println(", \033[1;32mPass\033[0m")
            DEBUG && !res && println(", \033[1;31mFail\033[0m")
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
