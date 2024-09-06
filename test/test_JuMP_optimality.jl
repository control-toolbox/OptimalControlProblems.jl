# test_JuMP_optimality
function test_JuMP_optimality()
    # Collecting all the OptimalControlProblems.JuMPModels models
    all_names = names(OptimalControlProblems, all=true)
    functions_list = filter(x -> isdefined(OptimalControlProblems, x) && 
                        isa(getfield(OptimalControlProblems, x), Function) &&
                        !startswith(string(x), "#") && 
                        !(x in [:eval, :include]), 
                        all_names)
    for f in functions_list
        @testset "$(f)" begin
            # Set up the model
            model = OptimalControlProblems.eval(f)(JuMPBackend())
            set_optimizer(model,Ipopt.Optimizer)
            set_silent(model)
            set_optimizer_attribute(model,"tol",1e-8)
            set_optimizer_attribute(model,"constr_viol_tol",1e-6)
            set_optimizer_attribute(model,"max_iter",500)
            set_optimizer_attribute(model,"mu_strategy","adaptive")
            set_optimizer_attribute(model,"linear_solver","mumps")
            set_optimizer_attribute(model, "max_wall_time", 120.0) 
            set_optimizer_attribute(model, "sb","yes")
            # Solve the model
            optimize!(model)
            # Test that the solver found an optimal solution
            if f == :truck_trailer || f == :quadrotor
                @test_broken termination_status(model) == MOI.LOCALLY_SOLVED 
            else
                @test termination_status(model) == MOI.LOCALLY_SOLVED 
            end
            
        end
    end    
end
