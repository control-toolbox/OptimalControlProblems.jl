using Printf

function test_quick()

    # comparison tolerances
    ε_rel_objective = 1e-4
    ε_abs_objective = 1e-6

    # options for solvers
    kwargs_ipopt = Dict(
        :print_level => 0,
        :tol => TOL,
        :mu_strategy => MU_STRATEGY,
        :sb => SB,
        :max_iter => MAX_ITER,
        :max_wall_time => MAX_WALL_TIME,
    )

    kwargs_madnlp = Dict(
        :print_level => MadNLP.ERROR,
        :tol => TOL,
        #:mu_strategy => MU_STRATEGY,
        #:sb => SB,
        :max_iter => MAX_ITER,
        :max_wall_time => MAX_WALL_TIME,
        :linear_solver => MumpsSolver,
    )

    max_r_err = -Inf # relative error max

    for f in LIST_OF_PROBLEMS
        grid_size = metadata(f)[:grid_size]

        @testset "$(string(f)) (objective)" verbose=VERBOSE begin
            DEBUG && println("\n", "┌─ ", string(f))
            DEBUG && println("│")

            ############### JuMP ###############
            nlp = OptimalControlProblems.eval(f)(JuMPBackend(); grid_size=grid_size)
            set_optimizer(nlp, Ipopt.Optimizer)
            set_silent(nlp)
            set_optimizer_attribute(nlp, "tol", TOL)
            set_optimizer_attribute(nlp, "max_iter", MAX_ITER)
            set_optimizer_attribute(nlp, "mu_strategy", MU_STRATEGY)
            set_optimizer_attribute(nlp, "linear_solver", "mumps")
            set_optimizer_attribute(nlp, "max_wall_time", MAX_WALL_TIME)
            set_optimizer_attribute(nlp, "sb", SB)
            optimize!(nlp)
            o_jp = objective_value(nlp)

            ########## OptimalControl ##########
            docp = OptimalControlProblems.eval(f)(OptimalControlBackend(); grid_size=grid_size)
            nlp = nlp_model(docp)
            nlp_sol = NLPModelsIpopt.ipopt(nlp; kwargs_ipopt...)
            sol = build_ocp_solution(docp, nlp_sol)
            o_oc = objective(sol)

            ########## OptimalControl_s ##########
            docp = OptimalControlProblems.eval(Symbol(f, :_s))(OptimalControlBackend(), :madnlp, :exa; grid_size=grid_size)
            nlp = nlp_model(docp)
            ocp = ocp_model(docp)
            nlp_sol = madnlp(nlp; kwargs_madnlp...)
            sol = build_ocp_solution(docp, nlp_sol)
            o_os = criterion(ocp) == :min ? objective(sol) : -objective(sol)

            ############### TEST ###############
            DEBUG && println("├─  objective")
            DEBUG && println("│")
            DEBUG && println("│     o_jp  = ", o_jp)
            DEBUG && println("│     o_oc  = ", o_oc)
            DEBUG && println("│     o_os  = ", o_os)
            DEBUG && println("│")

            # comparison JuMP and OptimalControl
            A = o_oc
            B = o_jp

            o_di = abs(A-B)
            o_bd = max(0.5*(abs(A) + abs(B))*ε_rel_objective, ε_abs_objective)
            max_r_err = max(max_r_err, o_di/(0.5*(abs(A) + abs(B))))

            DEBUG && println("│     JuMP vs OptimalControl")
            DEBUG && println("│")
            DEBUG && println("│          r_err = ", o_di/(0.5*(abs(A) + abs(B))))
            DEBUG && println("│          a_err = ", o_di)
            DEBUG && println("│          bound = ", o_bd)

            res = @my_test_broken o_di < o_bd

            DEBUG &&  res && println("│          \033[1;32mTest Passed\033[0m")
            DEBUG && !res && println("│          \033[1;31mTest Failed\033[0m")
            DEBUG && println("│")

            # comparison JuMP and OptimalControl_s
            A = o_os
            B = o_jp

            o_di = abs(A-B)
            o_bd = max(0.5*(abs(A) + abs(B))*ε_rel_objective, ε_abs_objective)
            max_r_err = max(max_r_err, o_di/(0.5*(abs(A) + abs(B))))

            DEBUG && println("│     JuMP vs OptimalControl_s")
            DEBUG && println("│")
            DEBUG && println("│          r_err = ", o_di/(0.5*(abs(A) + abs(B))))
            DEBUG && println("│          a_err = ", o_di)
            DEBUG && println("│          bound = ", o_bd)

            res = @my_test_broken o_di < o_bd

            DEBUG &&  res && println("│          \033[1;32mTest Passed\033[0m")
            DEBUG && !res && println("│          \033[1;31mTest Failed\033[0m")
            DEBUG && println("│")

            #
            DEBUG && println("└─")
        end
    end

    DEBUG && println("maximal relative error: ", max_r_err)
end
