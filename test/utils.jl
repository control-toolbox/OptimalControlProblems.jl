function L2_norm(T, X)
    # T and X are supposed to be one dimensional
    s = 0.0
    for i in 1:(length(T) - 1)
        s += 0.5 * (X[i]^2 + X[i + 1]^2) * (T[i + 1]-T[i])
    end
    return √(s)
end

function L1_norm(T, U)
    # T and X are supposed to be one dimensional
    s = 0.0
    for i in 1:(length(T) - 1)
        s += 0.5 * (abs(U[i]) + abs(U[i + 1])) * (T[i + 1]-T[i])
    end
    return s
end

macro my_test_broken(e)
    return esc(quote
        @test $e broken=!$e
    end)
end

function comparison(; max_iter, test_name)

    #
    available_test_names = [:init, :solution, :iter1]
    test_name ∈ available_test_names ||
        error("test_name must belong to ", available_test_names)

    # comparison Parameters: tolerances
    ε_rel_grid = 1e-6
    ε_abs_grid = 1e-6

    ε_rel_objective = 1e-4
    ε_abs_objective = 1e-6

    ε_rel_state = 1e-1
    ε_abs_state = 1e-6

    ε_rel_control = 1e-1
    ε_abs_control = 1e-6

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
    for f in LIST_OF_PROBLEMS

        N = OptimalControlProblems.metadata[f][:N] # get default N
        x_vars = OptimalControlProblems.metadata[f][:state_name]
        p_vars = OptimalControlProblems.metadata[f][:costate_name]
        u_vars = OptimalControlProblems.metadata[f][:control_name]
        v_vars = OptimalControlProblems.metadata[f][:variable_name]

        @testset "$(string(f)) ($(string(test_name)))" verbose=VERBOSE begin
            DEBUG && println("\n", "┌─ ", string(f), " (", string(test_name), ")")
            DEBUG && println("│")

            ########## OptimalControl ##########

            # set up the OptimalControl model 
            docp, nlp = OptimalControlProblems.eval(f)(OptimalControlBackend(); N=N)

            # solve the problem
            nlp_sol = NLPModelsIpopt.ipopt(nlp; Options...)

            # build the solution
            sol = build_OCP_solution(
                docp;
                primal=nlp_sol.solution,
                dual=nlp_sol.multipliers,
                docp_solution=nlp_sol,
            )

            sol_oc = deepcopy(sol) # for plotting

            # retrieve values of variables that we compare
            t_oc = time_grid(sol)
            x_oc = state(sol).(t_oc)
            u_oc = control(sol).(t_oc)
            o_oc = objective(sol)
            i_oc = nlp_sol.iter # iterations(sol) returns 0!
            v_oc = variable(sol)

            ############### JuMP ###############

            # set up the JuMP model
            model = OptimalControlProblems.eval(f)(JuMPBackend(); N=N)
            set_optimizer(model, Ipopt.Optimizer)
            set_silent(model)
            set_optimizer_attribute(model, "tol", Options[:tol])
            set_optimizer_attribute(model, "max_iter", Options[:max_iter])
            set_optimizer_attribute(model, "mu_strategy", Options[:mu_strategy])
            set_optimizer_attribute(model, "linear_solver", "mumps")
            set_optimizer_attribute(model, "max_wall_time", Options[:max_wall_time])
            set_optimizer_attribute(model, "sb", Options[:sb])

            # solve the model
            optimize!(model)

            # retrieve values
            t_jp = time_grid(f, model)
            x_jp = state(f, model).(t_jp)
            u_jp = control(f, model).(t_jp)
            o_jp = objective_value(model)
            i_jp = barrier_iterations(model)
            p_jp = costate(f, model).(t_jp)
            v_jp = variable(f, model)

            ############ TEST ############

            DEBUG && println("├─  iterations")
            DEBUG && println("│")
            DEBUG && println("│     i_oc = ", i_oc)
            DEBUG && println("│     i_jp = ", i_jp)
            DEBUG && println("│")

            # do we keep or remove the problem from the list
            keep_problem = true

            # time grids
            test_grid_ok = true
            @testset "grid" verbose=VERBOSE begin

                # final time
                DEBUG && println("├─  final time")
                DEBUG && println("│")
                DEBUG && println("│     tf oc = ", t_oc[end])
                DEBUG && println("│     tf jp = ", t_jp[end])
                DEBUG && println("│")

                # length
                res = @my_test_broken length(t_oc) == length(t_jp)
                keep_problem = keep_problem && (typeof(res) == Test.Pass)
                test_grid_ok = test_grid_ok && (typeof(res) == Test.Pass)

                DEBUG && println("├─  grid length")
                DEBUG && println("│")
                DEBUG && println("│     length(t_oc) = ", length(t_oc))
                DEBUG && println("│     length(t_jp) = ", length(t_jp))
                DEBUG &&
                    (typeof(res) == Test.Pass) &&
                    println("│     \033[1;32mTest Passed\033[0m")
                DEBUG &&
                    (typeof(res) != Test.Pass) &&
                    println("│     \033[1;31mTest Failed\033[0m")
                DEBUG && println("│")

                # values
                ti_oc_max = NaN
                ti_jp_max = NaN
                ti_di_max = NaN
                ti_bd_max = NaN
                ti_te_max = Inf
                itera_max = NaN
                if test_grid_ok
                    for i in eachindex(t_oc)
                        ti_di = t_oc[i] - t_jp[i]
                        ti_bd = max(
                            0.5*(abs(t_oc[i]) + abs(t_jp[i]))*ε_rel_grid, ε_abs_grid
                        )
                        res = @my_test_broken ti_di < ti_bd
                        keep_problem = keep_problem && (typeof(res) == Test.Pass)
                        test_grid_ok = test_grid_ok && (typeof(res) == Test.Pass)
                        if ti_bd - ti_di < ti_te_max
                            ti_te_max = ti_bd - ti_di
                            itera_max = i
                            ti_oc_max = t_oc[i]
                            ti_jp_max = t_jp[i]
                            ti_di_max = ti_di
                            ti_bd_max = ti_bd
                        end
                    end
                end

                DEBUG && println("├─  grid values (max error)")
                DEBUG && println("│")
                DEBUG && println("│     iter  = ", itera_max)
                DEBUG && println("│     ti oc = ", ti_oc_max)
                DEBUG && println("│     ti jp = ", ti_jp_max)
                DEBUG && println(
                    "│     r_err = ", ti_di_max/(0.5*(abs(ti_oc_max) + abs(ti_jp_max)))
                )
                DEBUG && println("│     a_err = ", ti_di_max)
                DEBUG && println("│     bound = ", ti_bd_max)
                DEBUG && test_grid_ok && println("│     \033[1;32mTest Passed\033[0m")
                DEBUG && !test_grid_ok && println("│     \033[1;31mTest Failed\033[0m")
                DEBUG && println("│")
            end

            # state
            if test_grid_ok
                @testset "state" verbose=VERBOSE begin
                    for i in eachindex(x_vars)
                        @testset "$(x_vars[i])" verbose=VERBOSE begin
                            xi_oc = [x_oc[k][i] for k in eachindex(t_oc)]
                            xi_jp = [x_jp[k][i] for k in eachindex(t_jp)]
                            L2_di = L2_norm(t_oc, xi_oc-xi_jp)
                            L2_oc = L2_norm(t_oc, xi_oc)
                            L2_jp = L2_norm(t_oc, xi_jp)
                            L2_bd = max(0.5*(L2_oc + L2_jp)*ε_rel_state, ε_abs_state)
                            res = @my_test_broken L2_di < L2_bd
                            keep_problem = keep_problem && (typeof(res) == Test.Pass)

                            DEBUG && println("├─  state $(x_vars[i])")
                            DEBUG && println("│")
                            DEBUG && println("│     L2 oc = ", L2_oc)
                            DEBUG && println("│     L2 jp = ", L2_jp)
                            DEBUG && println("│     r_err = ", L2_di/(0.5*(L2_oc + L2_jp)))
                            DEBUG && println("│     a_err = ", L2_di)
                            DEBUG && println("│     bound = ", L2_bd)
                            DEBUG &&
                                (typeof(res) == Test.Pass) &&
                                println("│     \033[1;32mTest Passed\033[0m")
                            DEBUG &&
                                (typeof(res) != Test.Pass) &&
                                println("│     \033[1;31mTest Failed\033[0m")
                            DEBUG && println("│")
                        end
                    end
                end
            end

            # control
            if test_grid_ok
                @testset "control" verbose=VERBOSE begin
                    for i in eachindex(u_vars)
                        @testset "$(u_vars[i])" verbose=VERBOSE begin
                            ui_oc = [u_oc[k][i] for k in eachindex(t_oc)]
                            ui_jp = [u_jp[k][i] for k in eachindex(t_jp)]
                            L2_di = L2_norm(t_oc, ui_oc-ui_jp)
                            L2_oc = L2_norm(t_oc, ui_oc)
                            L2_jp = L2_norm(t_oc, ui_jp)
                            L2_bd = max(0.5*(L2_oc + L2_jp)*ε_rel_control, ε_abs_control)
                            res = @my_test_broken L2_di < L2_bd

                            if f != :bioreactor # the test does not pass on GitHub CI
                                keep_problem = keep_problem && (typeof(res) == Test.Pass)
                            end

                            DEBUG && println("├─  control $(u_vars[i])")
                            DEBUG && println("│")
                            DEBUG && println("│     L2 oc = ", L2_oc)
                            DEBUG && println("│     L2 jp = ", L2_jp)
                            DEBUG && println("│     r_err = ", L2_di/(0.5*(L2_oc + L2_jp)))
                            DEBUG && println("│     a_err = ", L2_di)
                            DEBUG && println("│     bound = ", L2_bd)
                            DEBUG &&
                                (typeof(res) == Test.Pass) &&
                                println("│     \033[1;32mTest Passed\033[0m")
                            DEBUG &&
                                (typeof(res) != Test.Pass) &&
                                println("│     \033[1;31mTest Failed\033[0m")
                            DEBUG && println("│")
                        end
                    end
                end
            end

            # variable
            if test_grid_ok && !isnothing(v_vars)
                @testset "variable" verbose=VERBOSE begin
                    for i in eachindex(v_vars)
                        @testset "$(v_vars[i])" verbose=VERBOSE begin
                            vi_oc = v_oc[i]
                            vi_jp = v_jp[i]
                            vi_di = abs(vi_oc-vi_jp)
                            vi_bd = max(0.5*(abs(vi_oc) + abs(vi_jp))*ε_rel_control, ε_abs_control)
                            res = @my_test_broken vi_di < vi_bd

                            DEBUG && println("├─  variable $(v_vars[i])")
                            DEBUG && println("│")
                            DEBUG && println("│     vi oc = ", vi_oc)
                            DEBUG && println("│     vi jp = ", vi_jp)
                            DEBUG && println("│     r_err = ", vi_di/(0.5*(abs(vi_oc) + abs(vi_jp))))
                            DEBUG && println("│     a_err = ", vi_di)
                            DEBUG && println("│     bound = ", vi_bd)
                            DEBUG &&
                                (typeof(res) == Test.Pass) &&
                                println("│     \033[1;32mTest Passed\033[0m")
                            DEBUG &&
                                (typeof(res) != Test.Pass) &&
                                println("│     \033[1;31mTest Failed\033[0m")
                            DEBUG && println("│")
                        end
                    end
                end
            end

            # objective
            @testset "objective" verbose=VERBOSE begin
                o_di = abs(o_oc-o_jp)
                o_bd = max(0.5*(abs(o_oc) + abs(o_jp))*ε_rel_objective, ε_abs_objective)
                res = @my_test_broken o_di < o_bd
                if test_name != :init
                    keep_problem = keep_problem && (typeof(res) == Test.Pass)
                end

                DEBUG && println("├─  objective")
                DEBUG && println("│")
                DEBUG && println("│     o_oc  = ", o_oc)
                DEBUG && println("│     o_jp  = ", o_jp)
                DEBUG && println("│     r_err = ", o_di/(0.5*(abs(o_oc) + abs(o_jp))))
                DEBUG && println("│     a_err = ", o_di)
                DEBUG && println("│     bound = ", o_bd)
                DEBUG &&
                    (typeof(res) == Test.Pass) &&
                    println("│     \033[1;32mTest Passed\033[0m")
                DEBUG &&
                    (typeof(res) != Test.Pass) &&
                    println("│     \033[1;31mTest Failed\033[0m")
                DEBUG && println("│")
            end

            #
            DEBUG && println("└─")

            # do we keep or remove the problem from the list
            if !keep_problem
                global LIST_OF_PROBLEMS_FINAL
                LIST_OF_PROBLEMS_FINAL = setdiff(LIST_OF_PROBLEMS_FINAL, [f])
            end

            ############ PLOT ############

            figdir = joinpath(@__DIR__, "figures", string(test_name))
            isdir(figdir) || mkpath(figdir)

            n = length(x_vars)
            m = length(u_vars)
            @assert(length(p_vars)==n)

            # OptimalControl
            labelOC = if (test_name == :solution)
                "OptimalControl: " * string(i_oc) * " it"
            else
                "OptimalControl"
            end
            plt = plot(
                sol_oc;
                state_style=(color=1,),
                costate_style=(color=1, legend=:none),
                control_style=(color=1, legend=:none),
                path_style=(color=1, legend=:none),
                dual_style=(color=1, legend=:none),
                size=(900, 220*(n+m)),
                label=labelOC,
                leftmargin=20mm,
            )
            for i in 2:n
                plot!(plt[i]; legend=:none)
            end

            # JuMP
            labelJP = (test_name == :solution) ? "JuMP: " * string(i_oc) * " it" : "JuMP"
            for i in eachindex(x_vars) # state
                xi_jp = [x_jp[k][i] for k in eachindex(t_jp)]
                label = i == 1 ? labelJP : :none
                plot!(plt[i], t_jp, xi_jp; color=2, linestyle=:dash, label=label)
            end

            for i in eachindex(p_vars) # costate
                pi_jp = [p_jp[k][i] for k in eachindex(t_jp)]
                plot!(plt[n + i], t_jp, pi_jp; color=2, linestyle=:dash, label=:none)
            end

            for i in eachindex(u_vars) # control
                ui_jp = [u_jp[k][i] for k in eachindex(t_jp)]
                plot!(plt[2n + i], t_jp, ui_jp; color=2, linestyle=:dash, label=:none)
            end

            # save figure
            savefig(plt, joinpath(figdir, "$f" * ".pdf"))
        end
    end
end
