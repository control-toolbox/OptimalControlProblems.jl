using OptimalControl
using NLPModelsIpopt

# test_OptimalControl_optimality
function test_OptimalControl_optimality()
    nh=100
    # Collecting all the OptimalControlProblems.OptimalControlModels models
    all_names = names(OptimalControlProblems.OptimalControlModels, all=true)
    all_functions_list = filter(x -> isdefined(OptimalControlProblems.OptimalControlModels, x) && 
                        isa(getfield(OptimalControlProblems.OptimalControlModels, x), Function) &&
                        !startswith(string(x), "#") && 
                        !(x in [:eval, :include]), 
                        all_names)
    functions_list = filter(x -> !endswith(string(x), "_init"), all_functions_list)
    init_functions_list = filter(x -> endswith(string(x), "_init"), all_functions_list)
    for f in functions_list
        @testset "$(f)" begin
            # Set up the model
            model = OptimalControlProblems.OptimalControlModels.eval(f)()
            f_init = init_functions_list[findfirst(x -> x == Symbol(f , "_init"), init_functions_list)]
            init = OptimalControlProblems.OptimalControlModels.eval(f_init)(;nh=nh)
            sol = OptimalControl.solve(model, init=init, grid_size = nh; display = false)
            # Test that the solver found an optimal solution
            @test sol.message == "Solve_Succeeded"
        end
    end    
end