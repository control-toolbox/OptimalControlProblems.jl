# test_OptimalControl_optimality
function test_OptimalControl()
    kwargs = Dict(
        :print_level => 0,
        :tol => TOL,
        :mu_strategy => MU_STRATEGY,
        :sb => SB,
        :max_iter => MAX_ITER,
        :max_wall_time => MAX_WALL_TIME,
    )

    for f in LIST_OF_PROBLEMS
        @testset "$(f)" verbose=VERBOSE begin
            N = OptimalControlProblems.metadata[f][:N]

            # do we keep or remove the problem from the list
            keep_problem = true

            #
            DEBUG && println("\n", "┌─ ", string(f), " (OptimalControl)")
            DEBUG && println("│")

            # Set up the model
            docp = OptimalControlProblems.eval(f)(OptimalControlBackend(); N=N)
            nlp = nlp_model(docp)

            # Solve the model
            DEBUG && println("├─  Solve")
            DEBUG && println("│")
            print("  First solve:  ");
            @time sol = NLPModelsIpopt.ipopt(nlp; kwargs...)
            print("  Second solve: ");
            @time sol = NLPModelsIpopt.ipopt(nlp; kwargs...)
            DEBUG && println("│")

            # Infos
            DEBUG && println("├─  Infos")
            DEBUG && println("│")
            DEBUG && println("│     sol.status: ", sol.status)
            DEBUG && println("│     objective: ", sol.objective)
            DEBUG && println("│     iterations: ", sol.iter)
            DEBUG && println("│")

            # Test
            res = @my_test_broken (sol.status == :first_order || sol.status == :acceptable)
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
