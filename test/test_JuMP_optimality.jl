using Ipopt
using JuMP 

# test_JuMP_optimality
function test_JuMP_optimality()
    # Collecting all the OptimalControlProblems.JuMPModels models
    all_names = names(OptimalControlProblems.JuMPModels, all=true)
    functions_list = filter(x -> isdefined(OptimalControlProblems.JuMPModels, x) && 
                        isa(getfield(OptimalControlProblems.JuMPModels, x), Function) &&
                        !startswith(string(x), "#") && 
                        !(x in [:eval, :include]), 
                        all_names)
    for f in functions_list
        @testset "$(f)" begin
            # Set up the model
            model = OptimalControlProblems.JuMPModels.eval(f)()
            set_optimizer(model,Ipopt.Optimizer)
            set_silent(model)
            set_optimizer_attribute(model,"tol",1e-8)
            set_optimizer_attribute(model,"constr_viol_tol",1e-6)
            set_optimizer_attribute(model,"max_iter",1000)
            set_optimizer_attribute(model,"mu_strategy","adaptive")
            set_optimizer_attribute(model,"linear_solver","mumps")
            # Solve the model
            optimize!(model)
            # Test that the solver found an optimal solution
            @test termination_status(model) == MOI.LOCALLY_SOLVED
        end
    end    
end