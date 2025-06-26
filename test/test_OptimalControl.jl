# test_OptimalControl_optimality
function test_OptimalControl()

    kwargs = Dict(
        :print_level => 0,
        :tol => tol,
        :mu_strategy => mu_strategy,
        :sb => sb,
        :constr_viol_tol => constr_viol_tol,
        :max_iter => max_iter,
        :max_wall_time => max_wall_time,
    )

    for f in list_of_problems
        println("  $f:")
        @testset "$(f)" begin
            # Set up the model
            _, model = OptimalControlProblems.eval(f)(OptimalControlBackend())
            print("  First solve:  "); @time sol = NLPModelsIpopt.ipopt(model; kwargs...)
            print("  Second solve: "); @time sol = NLPModelsIpopt.ipopt(model; kwargs...)
            println("  sol.status = $(sol.status)\n")
            # Test that the solver found an optimal solution
            if  f == :truck_trailer ||
                f == :space_shuttle
                @test (sol.status == :infeasible) || (sol.status == :max_iter)
            else
                @test sol.status == :first_order
            end
        end
    end
end
