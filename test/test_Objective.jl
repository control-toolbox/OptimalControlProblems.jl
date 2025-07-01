function test_Objective()

    # Comparison Parameters
    ε = 1e-2

    # options for solvers
    kwargs = Dict(
        :print_level => 0,
        :tol => tol,
        :mu_strategy => mu_strategy,
        :sb => sb,
        :constr_viol_tol => constr_viol_tol,
        :max_iter => max_iter,
        :max_wall_time => max_wall_time,
    )

    println()
    println("\033[1m##################################\033[0m")
    println("\033[1m######### TEST OBJECTIVE #########\033[0m")
    println("\033[1m##################################\033[0m")
    println()

    for f in list_of_problems
        nh = OptimalControlProblems.metadata[f][:nh]
        @testset "$(f)" verbose = verbose begin
            println()
            println("############ TEST objective $f #############")

            ########## OptimalControl ##########

            # Set up the OptimalControl model 
            docp, OC_model = OptimalControlProblems.eval(f)(OptimalControlBackend())

            # Solve the problem
            nlp_sol = NLPModelsIpopt.ipopt(OC_model; kwargs...)

            # Build the solution
            sol = build_OCP_solution(docp; primal=nlp_sol.solution, dual=nlp_sol.multipliers)

            # Retrieves values of variables
            # x_oc = state(sol)
            # p_oc = costate(sol)
            # u_oc = control(sol)

            obj_oc = objective(sol)

            ############### JuMP ###############

            # Set up the JuMP model
            JuMP_model = OptimalControlProblems.eval(f)(JuMPBackend())
            set_optimizer(JuMP_model, Ipopt.Optimizer)
            set_silent(JuMP_model)
            set_optimizer_attribute(JuMP_model, "tol", tol)
            set_optimizer_attribute(JuMP_model, "constr_viol_tol", constr_viol_tol)
            set_optimizer_attribute(JuMP_model, "max_iter", max_iter)
            set_optimizer_attribute(JuMP_model, "mu_strategy", mu_strategy)
            set_optimizer_attribute(JuMP_model, "linear_solver", "mumps")
            set_optimizer_attribute(JuMP_model, "max_wall_time", max_wall_time)
            set_optimizer_attribute(JuMP_model, "sb", sb)

            # Solve the model
            optimize!(JuMP_model)

            obj_jmp = objective_value(JuMP_model)

            dist_obj = abs(obj_oc - obj_jmp)
            if !(dist_obj < ε)
                print("$dist_obj < $ε \033[1;33mTest Broken\033[0m\n")
                @test dist_obj < ε broken = true
                global list_of_problems_final
                list_of_problems_final = setdiff(list_of_problems_final, [f])
            else
                print("$dist_obj < $ε \033[1;32mTest Passed\033[0m\n")
                @test dist_obj < ε
            end
        end
    end

    println()
    println("\033[1m##################################\033[0m")
    println("\033[1m####### END TEST OBJECTIVE #######\033[0m")
    println("\033[1m##################################\033[0m")
    println()

end