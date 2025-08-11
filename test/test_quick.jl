using Printf

function test_quick()

    # comparison tolerance
    ε = 1e-2

    # options for solvers
    kwargs = Dict(
        :print_level => 0,
        :tol => TOL,
        :mu_strategy => MU_STRATEGY,
        :sb => SB,
        :max_iter => MAX_ITER,
        :max_wall_time => MAX_WALL_TIME,
    )

    for f in list_of_problems

        nh = OptimalControlProblems.metadata[f][:nh]
        print("$f ")
        
        ########## OptimalControl ##########
        docp, OC_model = OptimalControlProblems.eval(f)(OptimalControlBackend())
        nlp_sol = NLPModelsIpopt.ipopt(OC_model; kwargs...)
        sol = build_OCP_solution(docp; primal=nlp_sol.solution, dual=nlp_sol.multipliers, docp_solution=nlp_sol)
        obj_oc = objective(sol)

        ############### JuMP ###############
        JuMP_model = OptimalControlProblems.eval(f)(JuMPBackend())
        set_optimizer(JuMP_model, Ipopt.Optimizer)
        set_silent(JuMP_model)
        set_optimizer_attribute(JuMP_model, "tol", TOL)
        set_optimizer_attribute(JuMP_model, "max_iter", MAX_ITER)
        set_optimizer_attribute(JuMP_model, "mu_strategy", MU_STRATEGY)
        set_optimizer_attribute(JuMP_model, "linear_solver", "mumps")
        set_optimizer_attribute(JuMP_model, "max_wall_time", MAX_WALL_TIME)
        set_optimizer_attribute(JuMP_model, "sb", SB)
        optimize!(JuMP_model)
        obj_jmp = objective_value(JuMP_model)

        # objective relative error
        dist_obj = abs(obj_oc - obj_jmp) / (0.5 * abs(obj_oc + obj_jmp) + 1e-12)
        if dist_obj < ε
            @printf("Objective rel error %5.2g \033[1;32mTest Passed\033[0m\n", dist_obj)
        else
            @printf("Objective rel error %5.2g \033[1;33mTest Broken\033[0m JuMP: %5.2g vs OC: %5.2g\n", dist_obj, obj_jmp, obj_oc)
        end
        println("jmp: $obj_jmp")
        println("oc: $obj_oc")

    end

end