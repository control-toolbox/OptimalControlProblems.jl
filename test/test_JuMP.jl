# test_JuMP_optimality
function test_JuMP()
    # Collecting all the OptimalControlProblems.JuMPModels models
    all_names = names(OptimalControlProblems; all=true)
    functions_list = filter(
        x ->
            isdefined(OptimalControlProblems, x) &&
                isa(getfield(OptimalControlProblems, x), Function) &&
                !startswith(string(x), "#") &&
                !(x in [:eval, :include]),
        all_names,
    )

    pbs_with_issues = [:cart_pendulum]
    functions_list = setdiff(functions_list, pbs_with_issues)

    for f in functions_list
        @testset "$(f)" begin
            println("  $f:")
            # Set up the model
            model = OptimalControlProblems.eval(f)(JuMPBackend())
            set_optimizer(model, Ipopt.Optimizer)
            set_silent(model)
            set_optimizer_attribute(model, "tol", 1e-6)
            set_optimizer_attribute(model, "constr_viol_tol", 1e-6)
            set_optimizer_attribute(model, "max_iter", 1000)
            set_optimizer_attribute(model, "mu_strategy", "adaptive")
            set_optimizer_attribute(model, "linear_solver", "mumps")
            set_optimizer_attribute(model, "max_wall_time", 500.0)
            set_optimizer_attribute(model, "sb", "yes")
            # Solve the model
            print("  First solve:  "); @time optimize!(model)
            print("  Second solve: "); @time optimize!(model)
            # Test that the solver found an optimal solution
            println("  termination_status = $(termination_status(model))\n")
            if f == :truck_trailer || f == :quadrotor
                @test (termination_status(model) == MOI.LOCALLY_INFEASIBLE) || (termination_status(model) == MOI.ITERATION_LIMIT)
            else
                @test termination_status(model) == MOI.LOCALLY_SOLVED
            end
        end
    end
end
