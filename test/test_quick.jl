using Printf

function test_quick()

    # Comparison Parameters
    ε = 1e-2

    # options for solvers
    kwargs = Dict(
        :print_level => 0,
        :tol => tol,
        :mu_strategy => mu_strategy,
        :sb => sb,
        :max_iter => max_iter,
        :max_wall_time => max_wall_time,
    )

    for f in list_of_problems
        nh = OptimalControlProblems.metadata[f][:nh]
        print("$f ")
        ########## OptimalControl ##########
        docp, OC_model = OptimalControlProblems.eval(f)(OptimalControlBackend())
        nlp_sol = NLPModelsIpopt.ipopt(OC_model; kwargs...)
        sol = build_OCP_solution(docp; primal=nlp_sol.solution, dual=nlp_sol.multipliers)
        obj_oc = objective(sol)

        ############### JuMP ###############
        JuMP_model = OptimalControlProblems.eval(f)(JuMPBackend())
        set_optimizer(JuMP_model, Ipopt.Optimizer)
        set_silent(JuMP_model)
        set_optimizer_attribute(JuMP_model, "tol", tol)
        set_optimizer_attribute(JuMP_model, "max_iter", max_iter)
        set_optimizer_attribute(JuMP_model, "mu_strategy", mu_strategy)
        set_optimizer_attribute(JuMP_model, "linear_solver", "mumps")
        set_optimizer_attribute(JuMP_model, "max_wall_time", max_wall_time)
        set_optimizer_attribute(JuMP_model, "sb", sb)
        optimize!(JuMP_model)
        obj_jmp = objective_value(JuMP_model)

        # objective relative error
        dist_obj = abs(obj_oc - obj_jmp) / (obj_oc + obj_jmp) / 2
        if dist_obj < ε
            @printf("Objective rel error %5.2g \033[1;32mTest Passed\033[0m\n", dist_obj)
        else
            @printf("Objective rel error %5.2g \033[1;33mTest Broken\033[0m JuMP: %5.2g vs OC: %5.2g\n", dist_obj, obj_jmp, obj_oc)
        end

    end

end