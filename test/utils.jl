function L2_norm(T, X)
    # T and X are supposed to be one dimensional
    s = 0.0
    for i ∈ 1:(length(T)-1)
        s += 0.5 * (X[i]^2 + X[i+1]^2) * (T[i+1]-T[i])
    end
    return √(s)
end

macro my_test_broken(e)
    return esc(quote @test $e broken=!$e end)
end

function comparison(; max_iter, test_name)

    #
    available_test_names = [:init, :solution, :iter1]
    test_name ∈ available_test_names || error("test_name must belong to ", available_test_names)

    # Comparison Parameters
    ε_rel = 1e-2
    ε_abs = 1e-6

    # options for solvers
    Options = Dict(
        :print_level => 0,
        :tol => TOL,
        :mu_strategy => MU_STRATEGY,
        :sb => SB,
        :max_iter => max_iter,
        :max_wall_time => MAX_WALL_TIME,
    )

    # we loop over the problems
    for f in list_of_problems

        nh = OptimalControlProblems.metadata[f][:nh] # get default nh

        @testset "$(string(f)) ($(string(test_name)))" verbose=verbose begin

            debug && println("\n", "┌─ ", string(f), " (", string(test_name) ,")")
            debug && println("│")

            ########## OptimalControl ##########

            # Set up the OptimalControl model 
            docp, nlp = OptimalControlProblems.eval(f)(OptimalControlBackend(); nh=nh)

            # Solve the problem
            nlp_sol = NLPModelsIpopt.ipopt(nlp; Options...)

            # Build the solution
            sol = build_OCP_solution(docp; 
                primal=nlp_sol.solution, 
                dual=nlp_sol.multipliers,
                docp_solution=nlp_sol)

            sol_oc = deepcopy(sol) # for plotting

            # Retrieves values of variables that we compare
            t_oc = time_grid(sol)
            x_oc = state(sol).(t_oc)
            u_oc = control(sol).(t_oc)
            o_oc = objective(sol)

            ############### JuMP ###############

            # Set up the JuMP model
            model = OptimalControlProblems.eval(f)(JuMPBackend(); nh=nh)
            set_optimizer(model, Ipopt.Optimizer)
            set_silent(model)
            set_optimizer_attribute(model, "tol", Options[:tol])
            set_optimizer_attribute(model, "max_iter", Options[:max_iter])
            set_optimizer_attribute(model, "mu_strategy", Options[:mu_strategy])
            set_optimizer_attribute(model, "linear_solver", "mumps")
            set_optimizer_attribute(model, "max_wall_time", Options[:max_wall_time])
            set_optimizer_attribute(model, "sb", Options[:sb])

            # Solve the model
            optimize!(model)

            # Retrieves values of variables

            ## time grid: we assume that t0 = 0
            time_data, time_var_name, time_value = OptimalControlProblems.metadata[f][:time]
            
            t0 = 0
            t_jp = if time_data == "final_time"
                if time_value !== nothing
                    tf = time_value
                else
                    tf = value.(model[Symbol(time_var_name)])
                end
                range(t0, tf, nh+1)
            elseif time_data == "step"
                if time_value !== nothing
                    h = time_value
                    tf = h * nh
                    range(t0, tf, nh+1)
                else
                    h = value.(model[Symbol(time_var_name)])
                    if isa(h, Number)
                        tf = h * nh
                        range(t0, tf, nh+1)
                    else
                        cumsum([0, h...])
                    end
                end
            end

            ## state
            x_vars = OptimalControlProblems.metadata[f][:state_name]
            x_jp_vars = [JuMP.value.(model[Symbol(xv)]) for xv in x_vars]
            inds_x = axes(x_jp_vars[1], 1)
            x_jp = [[x_jp_vars[j][i] for j in 1:length(x_vars)] for i in inds_x]

            ## control
            u_vars = OptimalControlProblems.metadata[f][:control_name]
            u_jp_vars = [JuMP.value.(model[Symbol(uv)]) for uv in u_vars]
            inds_u = axes(u_jp_vars[1], 1)
            u_jp = [[u_jp_vars[j][i] for j in 1:length(u_vars)] for i in inds_u]

            ## objective
            o_jp = objective_value(model)

            ############ TEST ############

            # do we keep or remove the problem from the list
            keep_problem = true

            # time grids
            test_grid_ok = true
            @testset "grid" verbose=verbose begin

                debug && println("├─  grid")
                debug && println("│")
                debug && println("│     length(t_oc) = ", length(t_oc))
                debug && println("│     length(t_jp) = ", length(t_jp))

                # length
                res = @my_test_broken length(t_oc) == length(t_jp)
                keep_problem = keep_problem && (typeof(res) == Test.Pass)
                test_grid_ok = test_grid_ok && (typeof(res) == Test.Pass)

                # values
                if keep_problem
                    for i ∈ eachindex(t_oc)
                        t_di = t_oc[i] - t_jp[i]
                        t_bd = max(0.5*(abs(t_oc[i]) + abs(t_jp[i]))*ε_rel, ε_abs)
                        res = @my_test_broken t_di < t_bd
                        keep_problem = keep_problem && (typeof(res) == Test.Pass)
                        test_grid_ok = test_grid_ok && (typeof(res) == Test.Pass)
                    end
                end

                test_res = test_grid_ok ? "Passed" : "Failed"
                debug && println("│     \033[1;33mTest " * test_res * "\033[0m")
                debug && println("│")

            end

            # state
            if test_grid_ok
                @testset "state" verbose=verbose begin
                    for i ∈ eachindex(x_vars)
                        @testset "$(x_vars[i])" verbose=verbose begin
                            xi_oc = [ x_oc[k][i] for k ∈ eachindex(t_oc)]
                            xi_jp = [ x_jp[k][i] for k ∈ eachindex(t_jp)]
                            L2_di = L2_norm(t_oc, xi_oc-xi_jp)
                            L2_oc = L2_norm(t_oc, xi_oc)
                            L2_jp = L2_norm(t_oc, xi_jp)
                            L2_bd = max(0.5*(L2_oc + L2_jp)*ε_rel, ε_abs)
                            debug && println("├─  state $(x_vars[i])")
                            debug && println("│")
                            debug && println("│     L2 oc = ", L2_oc)
                            debug && println("│     L2 jp = ", L2_jp)
                            debug && println("│     error = ", L2_di)
                            debug && println("│     bound = ", L2_bd)
                            res = @my_test_broken L2_di < L2_bd
                            keep_problem = keep_problem && (typeof(res) == Test.Pass)
                            test_res = (typeof(res) == Test.Pass) ? "Passed" : "Failed"
                            debug && println("│     \033[1;33mTest " * test_res * "\033[0m")
                            debug && println("│")
                        end
                    end
                end
            end

            # control
            if test_grid_ok
                @testset "control" verbose=verbose begin
                    for i ∈ eachindex(u_vars)
                        @testset "$(u_vars[i])" verbose=verbose begin
                            ui_oc = [ u_oc[k][i] for k ∈ eachindex(t_oc)]
                            ui_jp = [ u_jp[k][i] for k ∈ eachindex(t_jp)]
                            L2_di = L2_norm(t_oc, ui_oc-ui_jp)
                            L2_oc = L2_norm(t_oc, ui_oc)
                            L2_jp = L2_norm(t_oc, ui_jp)
                            L2_bd = max(0.5*(L2_oc + L2_jp)*ε_rel, ε_abs)
                            debug && println("├─  control $(u_vars[i])")
                            debug && println("│")
                            debug && println("│     error = ", L2_di)
                            debug && println("│     L2 oc = ", L2_oc)
                            debug && println("│     L2 jp = ", L2_jp)
                            debug && println("│     bound = ", L2_bd)
                            res = @my_test_broken L2_di < L2_bd
                            keep_problem = keep_problem && (typeof(res) == Test.Pass)
                            test_res = (typeof(res) == Test.Pass) ? "Passed" : "Failed"
                            debug && println("│     \033[1;33mTest " * test_res * "\033[0m")
                            debug && println("│")
                        end
                    end
                end
            end

            # objective
            @testset "objective" verbose=verbose begin

                o_di = abs(o_oc-o_jp)
                o_bd = max(0.5*(abs(o_oc) + abs(o_jp))*ε_rel, ε_abs)

                debug && println("├─  objective")
                debug && println("│")
                debug && println("│     o_oc = ", o_oc)
                debug && println("│     o_jp = ", o_jp)
                debug && println("│     error = ", o_di)
                debug && println("│     bound = ", o_bd)
            
                res = @my_test_broken o_di < o_bd
                if test_name != :init
                    keep_problem = keep_problem && (typeof(res) == Test.Pass)
                end
                test_res = (typeof(res) == Test.Pass) ? "Passed" : "Failed"
                debug && println("│     \033[1;33mTest " * test_res * "\033[0m")
                debug && println("│")

            end

            #
            debug && println("└─")

            # do we keep or remove the problem from the list
            if !keep_problem
                global list_of_problems_final
                list_of_problems_final = setdiff(list_of_problems_final, [f])
            end

            ############ PLOT ############

            figdir = joinpath(@__DIR__, "figures", string(test_name))
            isdir(figdir) || mkpath(figdir)

            n = length(x_vars)
            m = length(u_vars)

            # OptimalControl
            plt = plot(sol_oc;
                state_style   = (color=1,),
                costate_style = (color=1,),
                control_style = (color=1,),
                size = (900, 220*(n+m)),
            )
            plot!(plt[1], [NaN]; color=1, label="OptimalControl")

            # JuMP
            for i ∈ eachindex(x_vars)
                xi_jp = [ x_jp[k][i] for k ∈ eachindex(t_jp)]
                label = i == 1 ? "JuMP" : :none
                plot!(plt[i], t_jp, xi_jp; color=2, linestyle=:dash, label=label)
            end

            for i ∈ eachindex(u_vars)
                ui_jp = [ u_jp[k][i] for k ∈ eachindex(t_jp)]
                plot!(plt[2n+i], t_jp, ui_jp; color=2, linestyle=:dash, label=:none)
            end

            # save figure
            savefig(plt, joinpath(figdir, "$f" * ".png"))

        end
    end

end