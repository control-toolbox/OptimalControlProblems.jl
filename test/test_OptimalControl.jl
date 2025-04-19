# test_OptimalControl_optimality
function test_OptimalControl()
    # Collecting all the OptimalControlProblems.OptimalControlModels models
    all_names = names(OptimalControlProblems; all=true)
    functions_list = filter(
        x ->
            isdefined(OptimalControlProblems, x) &&
                isa(getfield(OptimalControlProblems, x), Function) &&
                !startswith(string(x), "#") &&
                !(x in [:eval, :include]),
        all_names,
    )

    pbs_with_issues = [:glider]
    functions_list = setdiff(functions_list, pbs_with_issues)

    for f in functions_list
        println("  $f")
        @testset "$(f)" begin
            # Set up the model
            _, model = OptimalControlProblems.eval(f)(OptimalControlBackend())
            sol = NLPModelsIpopt.ipopt(
                model;
                print_level=0,
                tol=1e-8,
                mu_strategy="adaptive",
                sb="yes",
                constr_viol_tol=1e-6,
                max_iter=500,
                max_wall_time=240.0,
            )
            # Test that the solver found an optimal solution
            if  f == :moonlander    ||
                f == :truck_trailer ||
                f == :quadrotor     ||
                f == :space_shuttle
                @test (sol.status == :infeasible) || (sol.status == :max_iter)
            else
                @test sol.status == :first_order
            end
        end
    end
end
