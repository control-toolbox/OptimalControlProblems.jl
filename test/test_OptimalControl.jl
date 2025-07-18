# test_OptimalControl_optimality
function test_OptimalControl()

    kwargs = Dict(
        :print_level => 0,
        :tol => tol,
        :mu_strategy => mu_strategy,
        :sb => sb,
        :max_iter => max_iter,
        :max_wall_time => max_wall_time,
    )

    println()
    println("\033[1m#########################################\033[0m")
    println("\033[1m##### TEST CONVERGED OptimalControl #####\033[0m")
    println("\033[1m#########################################\033[0m")
    println()

    for f in list_of_problems
        println("  $f:")
        @testset "$(f)" verbose=verbose begin
            # Set up the model
            _, model = OptimalControlProblems.eval(f)(OptimalControlBackend()) # !+++ UPDATE
            print("  First solve:  "); @time sol = NLPModelsIpopt.ipopt(model; kwargs...)
            print("  Second solve: "); @time sol = NLPModelsIpopt.ipopt(model; kwargs...)
            println("  sol.status = $(sol.status)  objective (NLP) = $(sol.objective) \n")

            # Test that the solver found an optimal solution
            success = (sol.status == :first_order || sol.status == :first_order)
            if success
                @test success
                print("OptimalControl : $f converged : \033[1;32mTest Passed\033[0m\n")
            else 
                @test success broken=true
                print("OptimalControl : $f converged : \033[1;33mTest Broken\033[0m\n")
                global list_of_problems_final
                list_of_problems_final = setdiff(list_of_problems_final, [f])
            end
        end
    end

    println()
    println("\033[1m#########################################\033[0m")
    println("\033[1m### END TEST CONVERGED OptimalControl ###\033[0m")
    println("\033[1m#########################################\033[0m")
    println()

end
