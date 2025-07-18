function test_Comparison()

    # Comparison Parameters
    ε = 1e-2
    p = 2

    # options for solvers
    kwargs_init = Dict(
        :print_level => 0,
        :tol => tol,
        :mu_strategy => mu_strategy,
        :sb => sb,
        :constr_viol_tol => constr_viol_tol,
        :max_iter => 0,
        :max_wall_time => max_wall_time,
    )

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
    println("\033[1m###########################\033[0m")
    println("\033[1m####### COMPARISON ########\033[0m")
    println("\033[1m###########################\033[0m")
    println()

    for f in list_of_problems
        nh = OptimalControlProblems.metadata[f][:nh]
        @testset "$(string(f))" verbose = verbose begin
            println("############ TEST $f #############")
            println()

            #================== INIT ======================#

            ########## OptimalControl ##########

            # Set up the OptimalControl model 
            docp_init, OC_model_init = OptimalControlProblems.eval(f)(OptimalControlBackend()) # +++ UPDATE

            # Solve the problem
            nlp_sol_init = NLPModelsIpopt.ipopt(OC_model_init; kwargs_init...)

            # Build the solution
            sol_init = build_OCP_solution(docp_init; primal=nlp_sol_init.solution, dual=nlp_sol_init.multipliers)

            # Retrieves values of variables
            x_init_oc = state(sol_init)
            p_init_oc = costate(sol_init)
            u_init_oc = control(sol_init)

            ############### JuMP ###############

            # Set up the JuMP model
            JuMP_init_model = OptimalControlProblems.eval(f)(JuMPBackend())
            set_optimizer(JuMP_init_model, Ipopt.Optimizer)
            set_silent(JuMP_init_model)
            set_optimizer_attribute(JuMP_init_model, "tol", tol)
            set_optimizer_attribute(JuMP_init_model, "constr_viol_tol", constr_viol_tol)
            set_optimizer_attribute(JuMP_init_model, "max_iter", 0)
            set_optimizer_attribute(JuMP_init_model, "mu_strategy", mu_strategy)
            set_optimizer_attribute(JuMP_init_model, "linear_solver", "mumps")
            set_optimizer_attribute(JuMP_init_model, "max_wall_time", max_wall_time)
            set_optimizer_attribute(JuMP_init_model, "sb", sb)

            # Solve the model
            optimize!(JuMP_init_model)

            # Retrieves values of variables
            time_data, time_var_name, time_value = OptimalControlProblems.metadata[f][:time]
            if time_data == "final_time"
                if time_value !== nothing
                    tf_init = time_value
                else
                    tf_init = value.(JuMP_init_model[Symbol(time_var_name)])
                end
                h_init = tf_init / nh
            elseif time_data == "step"
                if time_value !== nothing
                    h_init = time_value
                else
                    h_init = value.(JuMP_init_model[Symbol(time_var_name)])
                end
                tf_init = h_init * nh
            end
            t_init = Vector((0:nh) * h_init)

            x_vars = OptimalControlProblems.metadata[f][:state_name]
            p_vars = OptimalControlProblems.metadata[f][:costate_name]
            u_vars = OptimalControlProblems.metadata[f][:control_name]

            x_jmp_vars_init = [JuMP.value.(JuMP_init_model[Symbol(xv)]) for xv in x_vars]
            inds_x_init = axes(x_jmp_vars_init[1], 1)
            x_jmp_init = [[x_jmp_vars_init[j][i] for j in 1:length(x_vars)] for i in inds_x_init]

            u_jmp_vars_init = [JuMP.value.(JuMP_init_model[Symbol(uv)]) for uv in u_vars]
            inds_u_init = axes(u_jmp_vars_init[1], 1)
            u_jmp_init = [[u_jmp_vars_init[j][i] for j in 1:length(u_vars)] for i in inds_u_init]

            p_jmp_vars_init = [JuMP.dual.(JuMP_init_model[Symbol(pv)]) for pv in p_vars]
            inds_p_init = axes(p_jmp_vars_init[1], 1)
            p_jmp_init = -[[p_jmp_vars_init[j][i] for j in 1:length(p_vars)] for i in inds_p_init]
            p_jmp_init = costateInterpolation(p_jmp_init, t_init)

            ############ TEST ############
            @testset "init" verbose=verbose begin 
            print("Init:\n")
                for k in 1:length(x_jmp_init[1])
                    dist_x_init = abs(x_init_oc(0)[k] - x_jmp_init[1][k])
                    print("  Test x$k : ")
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

                for k in 1:length(p_jmp_init[1])
                    dist_p_init = abs(p_init_oc(0)[k] - p_jmp_init[1][k])
                    print("  Test p$k : ")
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

                for k in 1:length(u_jmp_init[1])
                    dist_u_init = abs(u_init_oc(0)[k] - u_jmp_init[1][k])
                    print("  Test u$k : ")
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
            end 

            #======================= END INIT ========================#

            #======================= Lp + OBJECTIVE =================#

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
            obj_oc = nlp_sol.objective

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

            obj_jmp = objective_value(JuMP_model)

            dist_obj = abs(obj_oc - obj_jmp)
            @testset "objective" verbose=verbose begin
            print("Objective:\n")
            print("  Test objective : ")
                if !(dist_obj < ε)
                    print("$dist_obj < $ε \033[1;33mTest Broken\033[0m\n")
                    @test dist_obj < ε broken=true
                    global list_of_problems_final
                    list_of_problems_final = setdiff(list_of_problems_final, [f])
                else
                    print("$dist_obj < $ε \033[1;32mTest Passed\033[0m\n")
                    @test dist_obj < ε
                end
            end

            plots_x = Vector{Any}()
            plots_p = Vector{Any}()
            plots_u = Vector{Any}()

            @testset "norm_L$p" verbose=verbose begin
            print("Norm_L$p:\n")
                for k in 1:length(x_jmp[1])
                    dist_x_Lp = norm_Lp([x_oc((i - 1) * h)[k] - x_jmp[i][k] for i in 1:nh+1], p, h)
                    print("  Test x$k : ")
                    @testset "x$k" verbose = verbose begin
                        if !(dist_x_Lp < ε)
                            print("$dist_x_Lp < $ε \033[1;33mTest Broken\033[0m\n")
                            @test dist_x_Lp < ε broken=true
                            global list_of_problems_final
                            list_of_problems_final = setdiff(list_of_problems_final, [f])
                        else
                            print("$dist_x_Lp < $ε \033[1;32mTest Passed\033[0m\n")
                            @test dist_x_Lp < ε
                        end
                    end
                    px = plot(plot(sol)[k]; line=2, label="OptimalControl") # OptimalControl
                    px = plot!(t, [x_jmp[i][k] for i in 1:nh+1]; xlabel="t", ylabel=string(x_vars[k]), legend=false, line=2, color="red", linestyle=:dash, label="JuMP") # JuMP
                    push!(plots_x, px)
                end

                for k in 1:length(p_jmp[1])
                    dist_p_Lp = norm_Lp([p_oc((i - 1) * h)[k] - p_jmp[i][k] for i in 1:nh+1], p, h)
                    print("  Test p$k : ")
                    @testset "p$k" verbose = verbose begin
                        if !(dist_p_Lp < ε)
                            print("$dist_p_Lp < $ε \033[1;33mTest Broken\033[0m\n")
                            @test dist_p_Lp < ε broken=true
                            global list_of_problems_final
                            list_of_problems_final = setdiff(list_of_problems_final, [f])
                        else
                            print("$dist_p_Lp < $ε \033[1;32mTest Passed\033[0m\n")
                            @test dist_p_Lp < ε
                        end
                    end
                    pp = plot(plot(sol)[length(x_jmp[1])+k]; line=2, label="OptimalControl") # OptimalControl
                    pp = plot!(t, [p_jmp[i][k] for i in 1:nh+1]; xlabel="t", ylabel="p_" * string(x_vars[k]), legend=false, line=2, color="red", linestyle=:dash, label="JuMP") # JuMP
                    push!(plots_p, pp)
                end

                for k in 1:length(u_jmp[1])
                    dist_u_Lp = norm_Lp([u_oc((i - 1) * h)[k] - u_jmp[i][k] for i in 1:nh+1], p, h)
                    print("  Test u$k : ")
                    @testset "u$k" verbose = verbose begin
                        if !(dist_u_Lp < ε)
                            print("$dist_u_Lp < $ε \033[1;33mTest Broken\033[0m\n")
                            @test dist_u_Lp < ε broken=true
                            global list_of_problems_final
                            list_of_problems_final = setdiff(list_of_problems_final, [f])
                        else
                            print("$dist_u_Lp < $ε \033[1;32mTest Passed\033[0m\n")
                            @test dist_u_Lp < ε
                        end
                    end
                    pu = plot(plot(sol)[length(x_jmp[1])+length(p_jmp[1])+k]; line=2, label="OptimalControl") # OptimalControl
                    pu = plot!(t, [u_jmp[i][k] for i in 1:nh+1]; xlabel="t", ylabel=string(u_vars[k]), legend=false, line=2, color="red", linestyle=:dash, label="JuMP") # JuMP
                    push!(plots_u, pu)
                end
            end

            all_plots = vcat(plots_x, plots_p, plots_u)
            n = length(all_plots)

            figdir = joinpath(@__DIR__, "figures")
            isdir(figdir) || mkpath(figdir)

            fig = plot(all_plots..., layout=(n, 1), size=(900, 220 * n), suptitle="Comparison $f", titlefont=font(18))
            display(fig)

            savefig(fig, joinpath(figdir, "$f" * ".png"))

            println()
            println("############ END TEST $f #############")
            println()

            #================== END Lp + OBJECTIVE =====================#

        end
    end

    println()
    println("\033[1m###########################\033[0m")
    println("\033[1m##### END COMPARISON ######\033[0m")
    println("\033[1m###########################\033[0m")
    println()


end