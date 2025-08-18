using Printf

function test_quick()

    # comparison tolerances
    ε_rel_objective = 1e-4
    ε_abs_objective = 1e-6

    # options for solvers
    kwargs = Dict(
        :print_level => 0,
        :tol => TOL,
        :mu_strategy => MU_STRATEGY,
        :sb => SB,
        :max_iter => MAX_ITER,
        :max_wall_time => MAX_WALL_TIME,
    )

    max_r_err = -Inf # relative error max

    for f in LIST_OF_PROBLEMS
        N = OptimalControlProblems.metadata[f][:N]

        @testset "$(string(f)) (objective)" verbose=VERBOSE begin
            DEBUG && println("\n", "┌─ ", string(f))
            DEBUG && println("│")

            ########## OptimalControl ##########
            docp, nlp = OptimalControlProblems.eval(f)(OptimalControlBackend(); N=N)
            nlp_sol = NLPModelsIpopt.ipopt(nlp; kwargs...)
            sol = build_OCP_solution(
                docp;
                primal=nlp_sol.solution,
                dual=nlp_sol.multipliers,
                docp_solution=nlp_sol,
            )
            o_oc = objective(sol)

            ############### JuMP ###############
            model = OptimalControlProblems.eval(f)(JuMPBackend(); N=N)
            set_optimizer(model, Ipopt.Optimizer)
            set_silent(model)
            set_optimizer_attribute(model, "tol", TOL)
            set_optimizer_attribute(model, "max_iter", MAX_ITER)
            set_optimizer_attribute(model, "mu_strategy", MU_STRATEGY)
            set_optimizer_attribute(model, "linear_solver", "mumps")
            set_optimizer_attribute(model, "max_wall_time", MAX_WALL_TIME)
            set_optimizer_attribute(model, "sb", SB)
            optimize!(model)
            o_jp = objective_value(model)

            ############### TEST ###############
            # objective
            o_di = abs(o_oc-o_jp)
            o_bd = max(0.5*(abs(o_oc) + abs(o_jp))*ε_rel_objective, ε_abs_objective)

            DEBUG && println("├─  objective")
            DEBUG && println("│")
            DEBUG && println("│     o_oc  = ", o_oc)
            DEBUG && println("│     o_jp  = ", o_jp)
            DEBUG && println("│     r_err = ", o_di/(0.5*(abs(o_oc) + abs(o_jp))))
            DEBUG && println("│     a_err = ", o_di)
            DEBUG && println("│     bound = ", o_bd)

            res = @my_test_broken o_di < o_bd

            DEBUG &&
                (typeof(res) == Test.Pass) &&
                println("│     \033[1;32mTest Passed\033[0m")
            DEBUG &&
                (typeof(res) != Test.Pass) &&
                println("│     \033[1;31mTest Failed\033[0m")
            DEBUG && println("│")

            max_r_err = max(max_r_err, o_di/(0.5*(abs(o_oc) + abs(o_jp))))

            #
            DEBUG && println("└─")
        end
    end

    DEBUG && println("maximal relative error: ", max_r_err)
end
