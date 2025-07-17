# test_JuMP_optimality
function test_JuMP()

    for f in list_of_problems
        @testset "$(f)" verbose=verbose begin
            #print("  $f:")
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
            optimize!(model) # precompile
            #@time optimize!(model) #time
            @test termination_status(model) == MOI.LOCALLY_SOLVED
            if termination_status(model) == MOI.LOCALLY_SOLVED
                @test termination_status(model) == MOI.LOCALLY_SOLVED
                print("JuMP: $f \033[1;32mTest Passed\033[0m\n")
            else 
                @test termination_status(model) == MOI.LOCALLY_SOLVED broken=true
                print("JuMP : $f \033[1;33mTest Broken\033[0m\n")
                global list_of_problems_final
                list_of_problems_final = setdiff(list_of_problems_final, [f])
            end
        end
    end


end
