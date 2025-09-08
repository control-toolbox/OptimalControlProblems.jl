"""
L2_norm(T, X)

Compute the L² norm of a one-dimensional signal defined on a time grid.

# Arguments

- `T::AbstractVector`: Time grid, assumed one-dimensional and ordered.
- `X::AbstractVector`: Signal values at each time point, one-dimensional.

# Returns

- `::Float64`: The L² norm of `X` with respect to the grid `T`.

# Example

```julia-repl
julia> L2_norm(0:0.1:1, sin.(0:0.1:1))
0.5229090712505341
```
"""
function L2_norm(T, X)
    # T and X are supposed to be one dimensional
    s = 0.0
    for i in 1:(length(T) - 1)
        s += 0.5 * (X[i]^2 + X[i + 1]^2) * (T[i + 1]-T[i])
    end
    return √(s)
end

"""
@my_test_broken e

Mark a test as broken if the given expression fails.  
This macro wraps a test in the `@test` framework and sets `broken=!e`.

# Arguments

- `e::Expr`: The expression to be tested.

# Returns

- `::Expr`: An expression that expands into a `@test` with a `broken` flag.

# Example

```julia-repl
julia> @macroexpand @my_test_broken 1 == 2
:(@test 1 == 2 broken = !(1 == 2))
```
"""
macro my_test_broken(e)
    return esc(quote
        res = @test $e broken=!$e
        typeof(res) == Test.Pass
    end)
end

"""
comparison(; max_iter, test_name)

Run a comparison between the `OptimalControl` backend and a `JuMP` backend for a set of optimal control problems.  
The function validates solutions by comparing state, control, objective, and other quantities.

# Arguments

- `max_iter::Int`: Maximum number of solver iterations allowed.
- `test_name::Symbol`: The name of the test to run. Must be one of `:init`, `:solution`, or `:iter1`.

# Returns

- `::Nothing`: Runs the comparison tests and generates plots; does not return a value.

# Example

```julia-repl
julia> comparison(max_iter=100, test_name=:solution)
```
"""
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

    # options_ipopt for solvers
    options_ipopt = Dict(
        :print_level => 0,
        :tol => TOL,
        :mu_strategy => MU_STRATEGY,
        :sb => SB,
        :max_iter => max_iter,
        :max_wall_time => MAX_WALL_TIME,
    )

    options_madnlp = Dict(
        :print_level => MadNLP.ERROR,
        :tol => TOL,
        #:mu_strategy => MU_STRATEGY,
        #:sb => SB,
        :max_iter => max_iter,
        :max_wall_time => MAX_WALL_TIME,
        :linear_solver => MumpsSolver,
    )

    # we loop over the problems
    for f in LIST_OF_PROBLEMS
        N = metadata[f][:N] # get default N
        x_vars = metadata[f][:state_name]
        p_vars = metadata[f][:costate_name]
        u_vars = metadata[f][:control_name]
        v_vars = metadata[f][:variable_name]

        @testset "$(string(f)) ($(string(test_name)))" verbose=VERBOSE begin
            DEBUG && println("\n┌─ ", string(f), " (", string(test_name), ")")
            DEBUG && println("│")

            ############### JuMP ###############
            nlp_jp = OptimalControlProblems.eval(f)(JuMPBackend(); N=N)
            set_optimizer(nlp_jp, Ipopt.Optimizer)
            set_silent(nlp_jp)
            set_optimizer_attribute(nlp_jp, "tol", options_ipopt[:tol])
            set_optimizer_attribute(nlp_jp, "max_iter", options_ipopt[:max_iter])
            set_optimizer_attribute(nlp_jp, "mu_strategy", options_ipopt[:mu_strategy])
            set_optimizer_attribute(nlp_jp, "linear_solver", "mumps")
            set_optimizer_attribute(nlp_jp, "max_wall_time", options_ipopt[:max_wall_time])
            set_optimizer_attribute(nlp_jp, "sb", options_ipopt[:sb])
            optimize!(nlp_jp)

            t_jp = time_grid(f, nlp_jp)
            x_jp = state(f, nlp_jp).(t_jp)
            u_jp = control(f, nlp_jp).(t_jp)
            o_jp = objective(f, nlp_jp)
            i_jp = iterations(f, nlp_jp)
            v_jp = variable(f, nlp_jp)
            p_jp = costate(f, nlp_jp).(t_jp)

            ########## OptimalControl ##########
            docp = OptimalControlProblems.eval(f)(OptimalControlBackend(); N=N)
            nlp_oc = nlp_model(docp)
            nlp_sol = NLPModelsIpopt.ipopt(nlp_oc; options_ipopt...)
            sol_oc = build_ocp_solution(docp, nlp_sol)

            t_oc = time_grid(sol_oc)
            x_oc = state(sol_oc).(t_oc)
            u_oc = control(sol_oc).(t_oc)
            o_oc = objective(sol_oc)
            i_oc = iterations(sol_oc)
            v_oc = variable(sol_oc)

            ########## OptimalControl_s ##########
            docp = OptimalControlProblems.eval(Symbol(f, :_s))(OptimalControlBackend(), :madnlp, :exa; N=N)
            nlp_os = nlp_model(docp)
            nlp_sol = madnlp(nlp_os; options_madnlp...)
            sol_os = build_ocp_solution(docp, nlp_sol)

            t_os = time_grid(sol_os)
            x_os = state(sol_os).(t_os)
            u_os = control(sol_os).(t_os)
            o_os = objective(sol_os)
            i_os = iterations(sol_os)
            v_os = variable(sol_os)

            ########## Iterations ##########
            DEBUG && println("├─ Iterations     → JP: ", i_jp, ", OC: ", i_oc, ", OS: ", i_os)

            keep_problem = true

            ########## Variables / Constraints ##########
            @testset "nlp" verbose=VERBOSE begin
                nb_var_oc, nb_var_jp = get_nvar(nlp_oc), num_variables(nlp_jp)
                res = @my_test_broken nb_var_oc == nb_var_jp
                keep_problem = keep_problem && res
                DEBUG && @printf("├─ Variables      → OC: %d  JP: %d  %s\n", nb_var_oc, nb_var_jp,
                                res ? "\033[1;32mPASS\033[0m" : "\033[1;31mFAIL\033[0m")

                nb_con_oc = get_ncon(nlp_oc)
                nb_con_jp = num_constraints(nlp_jp; count_variable_in_set_constraints=false)
                res = @my_test_broken nb_con_oc == nb_con_jp
                keep_problem = keep_problem && res
                DEBUG && @printf("├─ Constraints    → OC: %d  JP: %d  %s\n", nb_con_oc, nb_con_jp,
                                res ? "\033[1;32mPASS\033[0m" : "\033[1;31mFAIL\033[0m")
            end

            ########## Time Grid ##########
            test_grid_ok = true
            @testset "grid" verbose=VERBOSE begin
                # final time
                tf_di = abs(t_oc[end] - t_jp[end])
                tf_bd = max(0.5*(t_oc[end]+t_jp[end])*ε_rel_grid, ε_abs_grid)
                res = @my_test_broken tf_di < tf_bd
                r_err = tf_di / (0.5*(t_oc[end]+t_jp[end]))
                DEBUG && @printf("├─ Final time     → OC: %.3e  JP: %.3e\n", t_oc[end], t_jp[end])
                DEBUG && @printf("│          r_err=%.3e  a_err=%.3e  bound=%.3e  %s\n",
                            r_err, tf_di, tf_bd,
                            res ? "\033[1;32mPASS\033[0m" : "\033[1;31mFAIL\033[0m")

                # length of the grids
                res = @my_test_broken length(t_oc) == length(t_jp)
                keep_problem = keep_problem && res
                test_grid_ok = test_grid_ok && res
                DEBUG && @printf("├─ Grid length    → OC: %d  JP: %d  %s\n", length(t_oc), length(t_jp),
                                res ? "\033[1;32mPASS\033[0m" : "\033[1;31mFAIL\033[0m")

                # max error
                if test_grid_ok
                    ti_di_max, ti_bd_max, itera_max = 0, NaN, 0
                    for i in eachindex(t_oc)
                        ti_di = t_oc[i] - t_jp[i]
                        ti_bd = max(0.5*(abs(t_oc[i])+abs(t_jp[i]))*ε_rel_grid, ε_abs_grid)
                        res = @my_test_broken abs(ti_di) < ti_bd
                        keep_problem = keep_problem && res
                        test_grid_ok = test_grid_ok && res
                        if abs(ti_di) ≥ abs(ti_di_max)
                            ti_di_max, ti_bd_max, itera_max = ti_di, ti_bd, i
                        end
                    end
                    r_err = abs(ti_di_max)/(0.5*(abs(t_oc[itera_max])+abs(t_jp[itera_max])))
                    DEBUG && @printf("├─ Grid max error → iter=%d  r_err=%.3e  a_err=%.3e  bound=%.3e  %s\n",
                                itera_max, r_err, abs(ti_di_max), ti_bd_max,
                                r_err<1.0 ? "\033[1;32mPASS\033[0m" : "\033[1;31mFAIL\033[0m")
                end
            end

            ########## States ##########
            if test_grid_ok
                @testset "state" verbose=VERBOSE begin
                    DEBUG && println("├─ States")
                    for i in eachindex(x_vars)
                        @testset "$(x_vars[i])" verbose=VERBOSE begin
                            xi_oc, xi_jp = [x_oc[k][i] for k in eachindex(t_oc)], [x_jp[k][i] for k in eachindex(t_jp)]
                            L2_di = L2_norm(t_oc, xi_oc - xi_jp)
                            L2_bd = max(0.5*(L2_norm(t_oc, xi_oc)+L2_norm(t_oc, xi_jp))*ε_rel_state, ε_abs_state)
                            res = @my_test_broken L2_di < L2_bd
                            keep_problem = keep_problem && res
                            r_err = L2_di / (0.5*(L2_norm(t_oc, xi_oc)+L2_norm(t_oc, xi_jp)))
                            DEBUG && @printf("│   %-6s r_err=%.3e  a_err=%.3e  bound=%.3e  %s\n",
                                        x_vars[i], r_err, L2_di, L2_bd,
                                        res ? "\033[1;32mPASS\033[0m" : "\033[1;31mFAIL\033[0m")
                        end
                    end
                end
            end

            ########## Controls ##########
            if test_grid_ok
                @testset "control" verbose=VERBOSE begin
                    DEBUG && println("├─ Controls")
                    for i in eachindex(u_vars)
                        @testset "$(u_vars[i])" verbose=VERBOSE begin
                            ui_oc, ui_jp = [u_oc[k][i] for k in eachindex(t_oc)], [u_jp[k][i] for k in eachindex(t_jp)]
                            L2_di = L2_norm(t_oc, ui_oc - ui_jp)
                            L2_bd = max(0.5*(L2_norm(t_oc, ui_oc)+L2_norm(t_oc, ui_jp))*ε_rel_control, ε_abs_control)
                            res = @my_test_broken L2_di < L2_bd
                            keep_problem = keep_problem && res
                            r_err = L2_di / (0.5*(L2_norm(t_oc, ui_oc)+L2_norm(t_oc, ui_jp)))
                            DEBUG && @printf("│   %-6s r_err=%.3e  a_err=%.3e  bound=%.3e  %s\n",
                                        u_vars[i], r_err, L2_di, L2_bd,
                                        res ? "\033[1;32mPASS\033[0m" : "\033[1;31mFAIL\033[0m")
                        end
                    end
                end
            end

            ########## Variables ##########
            if test_grid_ok && !isnothing(v_vars)
                @testset "variable" verbose=VERBOSE begin
                    DEBUG && println("├─ Variables")
                    for i in eachindex(v_vars)
                        @testset "$(v_vars[i])" verbose=VERBOSE begin
                            vi_oc, vi_jp = v_oc[i], v_jp[i]
                            vi_di = abs(vi_oc-vi_jp)
                            vi_bd = max(0.5*(abs(vi_oc)+abs(vi_jp))*ε_rel_control, ε_abs_control)
                            res = @my_test_broken vi_di < vi_bd
                            keep_problem = keep_problem && res
                            r_err = vi_di / (0.5*(abs(vi_oc)+abs(vi_jp)))
                            DEBUG && @printf("│   %-6s r_err=%.3e  a_err=%.3e  bound=%.3e  %s\n",
                                        v_vars[i], r_err, vi_di, vi_bd,
                                        res ? "\033[1;32mPASS\033[0m" : "\033[1;31mFAIL\033[0m")
                        end
                    end
                end
            end

            ########## Objective ##########
            @testset "objective" verbose=VERBOSE begin
                o_di = abs(o_oc-o_jp)
                o_bd = max(0.5*(abs(o_oc)+abs(o_jp))*ε_rel_objective, ε_abs_objective)
                res = @my_test_broken o_di < o_bd
                keep_problem = keep_problem && res
                r_err = o_di / (0.5*(abs(o_oc)+abs(o_jp)))
                DEBUG && println("├─ Objective")
                DEBUG && @printf("│          r_err=%.3e  a_err=%.3e  bound=%.3e  %s\n",
                            r_err, o_di, o_bd,
                            res ? "\033[1;32mPASS\033[0m" : "\033[1;31mFAIL\033[0m")
            end

            DEBUG && println("└─")

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
            color = 1
            labelOC = if (test_name == :solution)
                "OptimalControl: " * string(i_oc) * " it"
            else
                "OptimalControl"
            end
            plt = plot(
                sol_oc;
                state_style=(color=color,),
                costate_style=(color=color, legend=:none),
                control_style=(color=color, legend=:none),
                path_style=(color=color, legend=:none),
                dual_style=(color=color, legend=:none),
                size=(900, 220*(n+m)),
                label=labelOC,
                leftmargin=20mm,
            )
            for i in 2:n
                plot!(plt[i]; legend=:none)
            end

            # OptimalControl_s
            color = 2
            labelOC = if (test_name == :solution)
                "OptimalControl_s: " * string(i_oc) * " it"
            else
                "OptimalControl_s"
            end
            plot!(
                plt,
                sol_os;
                linestyle=:dot,
                state_style=(color=color,),
                costate_style=(color=color, legend=:none),
                control_style=(color=color, legend=:none),
                path_style=(color=color, legend=:none),
                dual_style=(color=color, legend=:none),
                size=(900, 220*(n+m)),
                label=labelOC,
                leftmargin=20mm,
            )
            for i in 2:n
                plot!(plt[i]; legend=:none)
            end

            # JuMP
            color = 3
            labelJP = (test_name == :solution) ? "JuMP: " * string(i_jp) * " it" : "JuMP"
            for i in eachindex(x_vars) # state
                xi_jp = [x_jp[k][i] for k in eachindex(t_jp)]
                label = i == 1 ? labelJP : :none
                plot!(plt[i], t_jp, xi_jp; color=color, linestyle=:dash, label=label)
            end

            for i in eachindex(p_vars) # costate
                pi_jp = [p_jp[k][i] for k in eachindex(t_jp)]
                plot!(plt[n + i], t_jp, -pi_jp; color=color, linestyle=:dash, label=:none)
            end

            for i in eachindex(u_vars) # control
                ui_jp = [u_jp[k][i] for k in eachindex(t_jp)]
                plot!(plt[2n + i], t_jp, ui_jp; color=color, linestyle=:dash, label=:none)
            end

            # save figure
            savefig(plt, joinpath(figdir, "$f" * ".pdf"))

        end# end testset
    end # end for
end
