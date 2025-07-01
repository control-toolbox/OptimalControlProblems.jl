# test_Comparison_JuMP_OptimalControl
function test_Comparison()

    # Comparison Parameters
    ε = 1e-2
    p = 2

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
    println("\033[1m###########################\033[0m")
    println("\033[1m### COMPARISON NORM L$p ###\033[0m")
    println("\033[1m###########################\033[0m")
    println()

    for f in list_of_problems
        nh = OptimalControlProblems.metadata[f][:nh]
        @testset "$(f)" verbose = verbose begin
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
            set_optimizer_attribute(JuMP_model, "max_iter", max_iter)
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

            plots_x = Vector{Any}()
            plots_p = Vector{Any}()
            plots_u = Vector{Any}()

            for k in 1:length(x_jmp[1])
                dist_x_Lp = norm_Lp([x_oc((i - 1) * h)[k] - x_jmp[i][k] for i in 1:nh+1], p, h)
                print("Test x$k : ")
                @testset "x$k" verbose = verbose begin
                    if !(dist_x_Lp < ε)
                        print("$dist_x_Lp < $ε \033[1;33mTest Broken\033[0m\n")
                        @test dist_x_Lp < ε broken = true
                        global list_of_problems_final
                        list_of_problems_final = setdiff(list_of_problems_final, [f])
                    else
                        print("$dist_x_Lp < $ε \033[1;32mTest Passed\033[0m\n")
                        @test dist_x_Lp < ε
                    end
                end
                px = plot(plot(sol)[k]; line=2, label="OptimalControl") # OptimalControl
                px = plot!(t, [x_jmp[i][k] for i in 1:nh+1]; xlabel="t", ylabel=x_vars[k], legend=false, line=2, color="red", linestyle=:dash, label="JuMP") # JuMP
                push!(plots_x, px)
            end

            for k in 1:length(p_jmp[1])
                dist_p_Lp = norm_Lp([p_oc((i - 1) * h)[k] - p_jmp[i][k] for i in 1:nh+1], p, h)
                print("Test p$k : ")
                @testset "p$k" verbose = verbose begin
                    if !(dist_p_Lp < ε)
                        print("$dist_p_Lp < $ε \033[1;33mTest Broken\033[0m\n")
                        @test dist_p_Lp < ε broken = true
                        global list_of_problems_final
                        list_of_problems_final = setdiff(list_of_problems_final, [f])
                    else
                        print("$dist_p_Lp < $ε \033[1;32mTest Passed\033[0m\n")
                        @test dist_p_Lp < ε
                    end
                end
                pp = plot(plot(sol)[length(x_jmp[1])+k]; line=2, label="OptimalControl") # OptimalControl
                pp = plot!(t, [p_jmp[i][k] for i in 1:nh+1]; xlabel="t", ylabel="p_" * x_vars[k], legend=false, line=2, color="red", linestyle=:dash, label="JuMP") # JuMP
                push!(plots_p, pp)
            end

            for k in 1:length(u_jmp[1])
                dist_u_Lp = norm_Lp([u_oc((i - 1) * h)[k] - u_jmp[i][k] for i in 1:nh+1], p, h)
                print("Test u$k : ")
                @testset "u$k" verbose = verbose begin
                    if !(dist_u_Lp < ε)
                        print("$dist_u_Lp < $ε \033[1;33mTest Broken\033[0m\n")
                        @test dist_u_Lp < ε broken = true
                        global list_of_problems_final
                        list_of_problems_final = setdiff(list_of_problems_final, [f])
                    else
                        print("$dist_u_Lp < $ε \033[1;32mTest Passed\033[0m\n")
                        @test dist_u_Lp < ε
                    end
                end
                pu = plot(plot(sol)[length(x_jmp[1])+length(p_jmp[1])+k]; line=2, label="OptimalControl") # OptimalControl
                pu = plot!(t, [u_jmp[i][k] for i in 1:nh+1]; xlabel="t", ylabel=u_vars[k], legend=false, line=2, color="red", linestyle=:dash, label="JuMP") # JuMP
                push!(plots_u, pu)
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

        end
    end

    println()
    println("\033[1m###############################\033[0m")
    println("\033[1m### END COMPARISON NORM L$p ###\033[0m")
    println("\033[1m###############################\033[0m")
    println()

end