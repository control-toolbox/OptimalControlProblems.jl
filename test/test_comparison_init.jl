function test_comparison_init()

    # Comparison Parameters
    ε = 1e-2

    # options for solvers
    kwargs = Dict(
        :print_level => 0,
        :tol => tol,
        :mu_strategy => mu_strategy,
        :sb => sb,
        :constr_viol_tol => constr_viol_tol,
        :max_iter => 0,
        :max_wall_time => max_wall_time,
    )

    println()
    println("\033[1m###########################\033[0m")
    println("\033[1m######### INIT ############\033[0m")
    println("\033[1m###########################\033[0m")
    println()

    for f in list_of_problems
        nh = OptimalControlProblems.metadata[f][:nh]
        @testset "$(f)" verbose=verbose begin
            println("############ TEST $f #############")
            println()

            ########## OptimalControl ##########

            # Set up the OptimalControl model 
            docp, OC_model = OptimalControlProblems.eval(f)(OptimalControlBackend())

            # Solve the problem
            nlp_sol = NLPModelsIpopt.ipopt(OC_model; kwargs...)

            # Build the solution
            sol = build_OCP_solution(docp; primal=nlp_sol.solution, dual=nlp_sol.multipliers)

            # Retrieves values of variables
            x_oc = state(sol)
            p_oc = costate(sol)
            u_oc = control(sol)

            ############### JuMP ###############

            # Set up the JuMP model
            JuMP_model = OptimalControlProblems.eval(f)(JuMPBackend())
            set_optimizer(JuMP_model, Ipopt.Optimizer)
            set_silent(JuMP_model)
            set_optimizer_attribute(JuMP_model, "tol", tol)
            set_optimizer_attribute(JuMP_model, "constr_viol_tol", constr_viol_tol)
            set_optimizer_attribute(JuMP_model, "max_iter", 0)
            set_optimizer_attribute(JuMP_model, "mu_strategy", mu_strategy)
            set_optimizer_attribute(JuMP_model, "linear_solver", "mumps")
            set_optimizer_attribute(JuMP_model, "max_wall_time", max_wall_time)
            set_optimizer_attribute(JuMP_model, "sb", sb)

            # Solve the model
            optimize!(JuMP_model)

            # Retrieves values of variables
            time_data, time_var_name, time_value = OptimalControlProblems.metadata[f][:time]
            if time_data == "final_time"
                if time_value !== nothing
                    tf = time_value
                else 
                    tf = value.(JuMP_model[Symbol(time_var_name)])
                end
                h = tf / nh
            elseif time_data == "step"
                if time_value !== nothing
                    h = time_value
                else 
                    h = value.(JuMP_model[Symbol(time_var_name)])
                end
                tf = h * nh
            end
            t = Vector((0:nh) * h)

            x_vars = OptimalControlProblems.metadata[f][:state_name]
            p_vars = OptimalControlProblems.metadata[f][:costate_name]
            u_vars = OptimalControlProblems.metadata[f][:control_name]

            x_jmp_vars = [JuMP.value.(JuMP_model[Symbol(xv)]) for xv in x_vars]
            inds_x = axes(x_jmp_vars[1], 1)
            x_jmp = [[x_jmp_vars[j][i] for j in 1:length(x_vars)] for i in inds_x]

            u_jmp_vars = [JuMP.value.(JuMP_model[Symbol(uv)]) for uv in u_vars]
            inds_u = axes(u_jmp_vars[1], 1)
            u_jmp = [[u_jmp_vars[j][i] for j in 1:length(u_vars)] for i in inds_u]

            p_jmp_vars = [JuMP.dual.(JuMP_model[Symbol(pv)]) for pv in p_vars]
            inds_p = axes(p_jmp_vars[1], 1)
            p_jmp = -[[p_jmp_vars[j][i] for j in 1:length(p_vars)] for i in inds_p]
            p_jmp = costateInterpolation(p_jmp, t)

            ############ TEST ############

            for k in 1:length(x_jmp[1])
                dist_x_init = abs(x_oc(0)[k] - x_jmp[1][k])
                print("Test x$k : ")
                @testset "x$k" verbose=verbose begin
                    if !(dist_x_init < ε)
                        print("$dist_x_init < $ε \033[1;33mTest Broken\033[0m\n")
                        @test dist_x_init < ε broken=true
                        global list_of_problems_final
                        list_of_problems_final = setdiff(list_of_problems_final, [f])
                    else
                        print("$dist_x_init < $ε \033[1;32mTest Passed\033[0m\n")
                        @test dist_x_init < ε
                    end
                end
            end
            
            for k in 1:length(p_jmp[1])
                dist_p_init = abs(p_oc(0)[k] - p_jmp[1][k])
                print("Test p$k : ")
                @testset "p$k" verbose=verbose begin
                    if !(dist_p_init < ε)
                        print("$dist_p_init < $ε \033[1;33mTest Broken\033[0m\n")
                        @test dist_p_init < ε broken=true
                    else
                        print("$dist_p_init < $ε \033[1;32mTest Passed\033[0m\n")
                        @test dist_p_init < ε
                    end
                end
            end

            for k in 1:length(u_jmp[1])
                dist_u_init = abs(u_oc(0)[k] - u_jmp[1][k])
                print("Test u$k : ")
                @testset "u$k" verbose=verbose begin
                    if !(dist_u_init < ε)
                        print("$dist_u_init < $ε \033[1;33mTest Broken\033[0m\n")
                        @test dist_u_init < ε broken=true
                        global list_of_problems_final
                        list_of_problems_final = setdiff(list_of_problems_final, [f])
                    else
                        print("$dist_u_init < $ε \033[1;32mTest Passed\033[0m\n")
                        @test dist_u_init < ε
                    end
                end
            end

            println()
            println("############ END TEST $f #############")
            println()

        end
    end

    println()
    println("\033[1m###########################\033[0m")
    println("\033[1m######## END INIT #########\033[0m")
    println("\033[1m###########################\033[0m")
    println()

end