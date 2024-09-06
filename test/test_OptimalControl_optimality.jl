using CTBase
using CTDirect
using NLPModelsIpopt

# test_OptimalControl_optimality
function test_OptimalControl_optimality()
    # Collecting all the OptimalControlProblems.OptimalControlModels models
    all_names = names(OptimalControlProblems, all=true)
    functions_list = filter(x -> isdefined(OptimalControlProblems, x) && 
                        isa(getfield(OptimalControlProblems, x), Function) &&
                        !startswith(string(x), "#") && 
                        !(x in [:eval, :include]), 
                        all_names)
    for f in functions_list
        @testset "$(f)" begin
            # Set up the model
            model = OptimalControlProblems.eval(f)(OptimalControlBackend())
            sol = NLPModelsIpopt.ipopt(model; print_level=0, tol=1e-8, mu_strategy="adaptive", sb="yes", constr_viol_tol=1e-6, max_iter=500, max_wall_time=120.0)
            # Test that the solver found an optimal solution
            if f == :moonlander || f == :quadrotor || f == :truck_trailer || f == :space_shuttle
                @test_broken sol.status ==  :first_order
            else
                @test sol.status == :first_order
            end
        end
    end    
end