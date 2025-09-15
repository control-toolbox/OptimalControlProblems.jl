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

    test_tag(res) = res ? "\033[1;32mPASS\033[0m" : "\033[1;31mFAIL\033[0m"

    #
    available_test_names = [:init, :solution, :iter1]
    test_name ∈ available_test_names ||
        error("test_name must belong to ", available_test_names)

    # comparison Parameters: tolerances
    ε_rel_grid = 1e-2
    ε_abs_grid = 1e-6

    ε_rel_objective = 1e-4
    ε_abs_objective = 1e-6

    ε_rel_state = 1e-1
    ε_abs_state = 1e-6

    ε_rel_control = 1e-1
    ε_abs_control = 1e-6

    ε_rel_variable = 1e-1
    ε_abs_variable = 1e-6

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

    #
    function test_L2_i(i, times, A, B, A_name, B_name; ε_abs, ε_rel)
        yi_oc, yi_jp = [B[k][i] for k in eachindex(times)], [A[k][i] for k in eachindex(times)]
        L2_di = L2_norm(times, yi_oc - yi_jp)
        L2_bd = max(0.5*(L2_norm(times, yi_oc)+L2_norm(times, yi_jp))*ε_rel, ε_abs)
        res = @my_test_broken L2_di < L2_bd
        r_err = L2_di / (0.5*(L2_norm(times, yi_oc)+L2_norm(times, yi_jp)))
        DEBUG && @printf("│      → %s vs %s: r_err=%.3e  a_err=%.3e  bound=%.3e  %s\n", A_name, B_name, r_err, L2_di, L2_bd, test_tag(res))
        return res
    end

    function test_L2_i(i, times, A, B, A_name, B_name, keep_problem; ε_abs, ε_rel)
        res = test_L2_i(i, times, A, B, A_name, B_name; ε_abs=ε_abs, ε_rel=ε_rel)
        return keep_problem && res
    end

    #
    function test_abs(A, B, A_name, B_name; ε_abs, ε_rel)
        tf_di = abs(A - B)
        tf_bd = max(0.5*(A+B)*ε_rel, ε_abs)
        res = @my_test_broken tf_di < tf_bd
        r_err = tf_di / (0.5*(A+B))
        DEBUG && @printf("│      → %s: %.3e  %s: %.3e  r_err=%.3e  a_err=%.3e  bound=%.3e  %s\n", A_name, A, B_name, B, r_err, tf_di, tf_bd, test_tag(res))
        return res
    end

    function test_abs(A, B, A_name, B_name, keep_problem; ε_abs, ε_rel)
        res = test_abs(A, B, A_name, B_name; ε_abs=ε_abs, ε_rel=ε_rel)
        return res && keep_problem
    end

    function test_abs(A, B, A_name, B_name, keep_problem, test_grid_ok; ε_abs, ε_rel)
        res = test_abs(A, B, A_name, B_name; ε_abs=ε_abs, ε_rel=ε_rel)
        return res && keep_problem, res && test_grid_ok
    end

    function test_abs_i(i, A, B, A_name, B_name; ε_abs, ε_rel)
        return test_abs(A[i], B[i], A_name, B_name; ε_abs, ε_rel)
    end
     
    function test_abs_i(i, A, B, A_name, B_name, keep_problem; ε_abs, ε_rel)
        return test_abs(A[i], B[i], A_name, B_name, keep_problem; ε_abs, ε_rel)
    end

    #
    function test_int(A, B, A_name, B_name)
        res = @my_test_broken A == B
        DEBUG && @printf("│      → %s: %d  %s: %d  %s\n", A_name, A, B_name, B, test_tag(res))
        return res
    end

    function test_int(A, B, A_name, B_name, keep_problem)
        res = test_int(A, B, A_name, B_name)
        return res && keep_problem
    end

    function test_int(A, B, A_name, B_name, keep_problem, test_grid_ok)
        res = test_int(A, B, A_name, B_name)
        return res && keep_problem, res && test_grid_ok
    end

    #
    function test_length(A, B, A_name, B_name)
        return test_int(length(A), length(B), A_name, B_name)
    end

    function test_length(A, B, A_name, B_name, keep_problem, test_grid_ok)
        res = test_length(A, B, A_name, B_name)
        return res && keep_problem, res && test_grid_ok
    end
    
    #
    function test_grid_max_error(A, B, A_name, B_name, keep_problem, test_grid_ok)
        ti_di_max, ti_bd_max, itera_max = 0, NaN, 0
        for i in eachindex(B)
            ti_di = B[i] - A[i]
            ti_bd = max(0.5*(abs(B[i])+abs(A[i]))*ε_rel_grid, ε_abs_grid)
            res = @my_test_broken abs(ti_di) < ti_bd
            keep_problem = keep_problem && res
            test_grid_ok = test_grid_ok && res
            if abs(ti_di) ≥ abs(ti_di_max)
                ti_di_max, ti_bd_max, itera_max = ti_di, ti_bd, i
            end
        end
        r_err = abs(ti_di_max)/(0.5*(abs(B[itera_max])+abs(A[itera_max])))
        res = r_err<1.0
        DEBUG && @printf("│      → %s vs %s: iter=%d  r_err=%.3e  a_err=%.3e  bound=%.3e  %s\n", A_name, B_name, itera_max, r_err, abs(ti_di_max), ti_bd_max, test_tag(res))
        return keep_problem && res, test_grid_ok && res
    end

    # we loop over the problems
    for f in LIST_OF_PROBLEMS
        grid_size = metadata(f)[:grid_size] # get default number of steps
        x_vars = metadata(f)[:state_name]
        p_vars = metadata(f)[:costate_name]
        u_vars = metadata(f)[:control_name]
        v_vars = metadata(f)[:variable_name]

        @testset "$(string(f)) ($(string(test_name)))" verbose=VERBOSE begin
            DEBUG && println("\n┌─ ", string(f), " (", string(test_name), ")")
            DEBUG && println("│")

            ############### JuMP ###############
            nlp_jp = OptimalControlProblems.eval(f)(JuMPBackend(); grid_size=grid_size)
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
            nb_var_jp = num_variables(nlp_jp)
            nb_con_jp = num_constraints(nlp_jp; count_variable_in_set_constraints=false)

            ########## OptimalControl ##########
            docp = OptimalControlProblems.eval(f)(OptimalControlBackend(); grid_size=grid_size)
            nlp_oc = nlp_model(docp)
            nlp_sol = NLPModelsIpopt.ipopt(nlp_oc; options_ipopt...)
            sol_oc = build_ocp_solution(docp, nlp_sol)

            t_oc = time_grid(sol_oc)
            x_oc = state(sol_oc).(t_oc)
            u_oc = control(sol_oc).(t_oc)
            o_oc = objective(sol_oc)
            i_oc = iterations(sol_oc)
            v_oc = variable(sol_oc)
            nb_var_oc = get_nvar(nlp_oc)
            nb_con_oc = get_ncon(nlp_oc)

            ########## OptimalControl_s ##########
            model_backend = :exa # :adnlp
            docp = OptimalControlProblems.eval(Symbol(f, :_s))(OptimalControlBackend(), :madnlp, model_backend; grid_size=grid_size)
            nlp_os = nlp_model(docp)
            nlp_sol = madnlp(nlp_os; options_madnlp...)
            sol_os = build_ocp_solution(docp, nlp_sol)

            t_os = time_grid(sol_os)
            x_os = state(sol_os).(t_os)
            u_os = control(sol_os).(t_os)
            o_os = objective(sol_os)
            i_os = iterations(sol_os)
            v_os = variable(sol_os)
            nb_var_os = get_nvar(nlp_os)
            nb_con_os = get_ncon(nlp_os)

            ########## Iterations ##########
            DEBUG && @printf("├─ Iterations\n")
            DEBUG && @printf("│      → JP: %d  OC: %d  OS: %d\n", i_jp, i_oc, i_os)

            keep_problem = true

            ########## Variables / Constraints ##########
            @testset "nlp" verbose=VERBOSE begin
                DEBUG && @printf("├─ Variables\n")
                keep_problem = test_int(nb_var_jp, nb_var_oc, "JP", "OC", keep_problem)
                keep_problem = test_int(nb_var_jp, nb_var_os, "JP", "OS", keep_problem)

                DEBUG && @printf("├─ Constraints\n")
                keep_problem = test_int(nb_con_jp, nb_con_oc, "JP", "OC", keep_problem)
                keep_problem = test_int(nb_con_jp, nb_con_os, "JP", "OS", keep_problem)
            end

            ########## Time Grid ##########
            test_grid_ok = true
            @testset "grid" verbose=VERBOSE begin
                
                # ----------------------------
                # final time
                DEBUG && @printf("├─ Final time\n")
                keep_problem, test_grid_ok = test_abs(t_jp[end], t_oc[end], "JP", "OC", keep_problem, test_grid_ok; ε_abs=ε_abs_grid, ε_rel=ε_rel_grid)
                keep_problem, test_grid_ok = test_abs(t_jp[end], t_os[end], "JP", "OS", keep_problem, test_grid_ok; ε_abs=ε_abs_grid, ε_rel=ε_rel_grid)

                # ----------------------------
                # length of the grids
                DEBUG && @printf("├─ Grid length\n")
                keep_problem, test_grid_ok = test_length(t_jp, t_oc, "JP", "OC", keep_problem, test_grid_ok)
                keep_problem, test_grid_ok = test_length(t_jp, t_os, "JP", "OS", keep_problem, test_grid_ok)

                # ----------------------------
                # max error
                if test_grid_ok
                    DEBUG && @printf("├─ Grid max error\n")
                    keep_problem, test_grid_ok = test_grid_max_error(t_jp, t_oc, "JP", "OC", keep_problem, test_grid_ok)
                    keep_problem, test_grid_ok = test_grid_max_error(t_jp, t_os, "JP", "OS", keep_problem, test_grid_ok)
                end
            end

            ########## States ##########
            if test_grid_ok
                @testset "state" verbose=VERBOSE begin
                    DEBUG && println("├─ States")
                    for i in eachindex(x_vars)
                        DEBUG && @printf("│   %-6s\n", x_vars[i])
                        @testset "$(x_vars[i])" verbose=VERBOSE begin
                            keep_problem = test_L2_i(i, t_jp, x_jp, x_oc, "JP", "OC", keep_problem; ε_abs=ε_abs_state, ε_rel=ε_rel_state)
                            keep_problem = test_L2_i(i, t_jp, x_jp, x_os, "JP", "OS", keep_problem; ε_abs=ε_abs_state, ε_rel=ε_rel_state)
                        end
                    end
                end
            end

            ########## Controls ##########
            if test_grid_ok
                @testset "control" verbose=VERBOSE begin
                    DEBUG && println("├─ Controls")
                    for i in eachindex(u_vars)
                        DEBUG && @printf("│   %-6s\n", u_vars[i])
                        @testset "$(u_vars[i])" verbose=VERBOSE begin
                            keep_problem = test_L2_i(i, t_jp, u_jp, u_oc, "JP", "OC", keep_problem; ε_abs=ε_abs_control, ε_rel=ε_rel_control)
                            keep_problem = test_L2_i(i, t_jp, u_jp, u_os, "JP", "OS", keep_problem; ε_abs=ε_abs_control, ε_rel=ε_rel_control)
                        end
                    end
                end
            end

            ########## Variables ##########
            if test_grid_ok && !isnothing(v_vars)
                @testset "variable" verbose=VERBOSE begin
                    DEBUG && println("├─ Variables")
                    for i in eachindex(v_vars)
                        DEBUG && @printf("│   %-6s\n", v_vars[i])
                        @testset "$(v_vars[i])" verbose=VERBOSE begin
                            keep_problem = test_abs_i(i, v_jp, v_oc, "JP", "OC", keep_problem; ε_abs=ε_abs_variable, ε_rel=ε_rel_variable)
                            keep_problem = test_abs_i(i, v_jp, v_os, "JP", "OS", keep_problem; ε_abs=ε_abs_variable, ε_rel=ε_rel_variable)
                        end
                    end
                end
            end

            ########## Objective ##########
            DEBUG && println("├─ Objective")
            @testset "objective" verbose=VERBOSE begin
                keep_problem = test_abs(o_jp, o_oc, "JP", "OC", keep_problem; ε_abs=ε_abs_objective, ε_rel=ε_rel_objective)
                keep_problem = test_abs(o_jp, o_os, "JP", "OS", keep_problem; ε_abs=ε_abs_objective, ε_rel=ε_rel_objective)
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
                label=labelOC,
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
