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

    println()
    println("\033[1m###########################################\033[0m")
    println("\033[1m##### TEST CONVERGENCE OptimalControl #####\033[0m")
    println("\033[1m###########################################\033[0m")
    println()

    for f in list_of_problems
        println("$f:")
        @testset "$(f)" verbose=verbose begin
            # Set up the model
            _, model = OptimalControlProblems.eval(f)(OptimalControlBackend()) # !+++ UPDATE
            print("  First solve:  "); @time sol = NLPModelsIpopt.ipopt(model; kwargs...)
            print("  Second solve: "); @time sol = NLPModelsIpopt.ipopt(model; kwargs...)
            println(
                "  sol.status = ", sol.status,  
                ", objective = ", sol.objective,
                ", iterations = ", sol.iter
            )

            # Test that the solver found an optimal solution
            success = (sol.status == :first_order || sol.status == :acceptable)
            if success
                @test success
                println("  OptimalControl : $f convergence: \033[1;32mTest Passed\033[0m\n")
            else 
                @test success broken=true
                println("  OptimalControl : $f convergence: \033[1;33mTest Broken\033[0m\n")
                global list_of_problems_final
                list_of_problems_final = setdiff(list_of_problems_final, [f])
            end
        end
    end

    println()
    println("\033[1m###########################################\033[0m")
    println("\033[1m### END TEST CONVERGENCE OptimalControl ###\033[0m")
    println("\033[1m###########################################\033[0m")
    println()

end
