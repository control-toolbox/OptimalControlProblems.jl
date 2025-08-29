function generate_documentation(
    PROBLEM::String, DESCRIPTION::String; draft::Union{Bool,Nothing}
)
    TITLE = uppercasefirst(replace(PROBLEM, "_" => " "))

    DRAFT = if isnothing(draft)
        ""
    elseif draft
        """
        ```@meta
        Draft = true
        ```
        """
    else
        """
        ```@meta
        Draft = false
        ```
        """
    end

    documentation=DRAFT * """
    # $TITLE

    $DESCRIPTION

    ## Packages

    Import all necessary packages and define DataFrames to store information about the problem and resolution results.

    ```@example main
    using OptimalControlProblems    # to access the Beam model
    using OptimalControl            # to import the OptimalControl model
    using NLPModelsIpopt            # to solve the model with Ipopt
    import DataFrames: DataFrame    # to store data
    using NLPModels                 # to retrieve data from the NLP solution
    using Plots                     # to plot the trajectories
    using Plots.PlotMeasures        # for leftmargin, bottommargin
    using JuMP                      # to import the JuMP model
    using Ipopt                     # to solve the JuMP model with Ipopt

    data_pb = DataFrame(            # to store data about the problem
        Problem=Symbol[],
        Grid_Size=Int[],
        Variables=Int[],
        Constraints=Int[],
    )

    data_re = DataFrame(            # to store data about the resolutions
        Model=Symbol[],
        Flag=Any[],
        Iterations=Int[],
        Objective=Float64[],
    )
    nothing # hide
    ```

    ## Initial guess

    The initial guess (or first iterate) can be visualised by running the solver with `max_iter=0`. Here is the initial guess.

    ```@raw html
    <details><summary>Click to unfold and see the code for plotting the initial guess.</summary>
    ```

    ```@example main
    function plot_initial_guess(problem)

        # dimensions
        x_vars = metadata[problem][:state_name]
        u_vars = metadata[problem][:control_name]
        n = length(x_vars) # number of states
        m = length(u_vars) # number of controls

        # import OptimalControl model
        docp = eval(problem)(OptimalControlBackend())
        nlp_oc = nlp_model(docp)

        # solve
        nlp_oc_sol = NLPModelsIpopt.ipopt(nlp_oc; max_iter=0)

        # build an optimal control solution
        ocp_sol = build_ocp_solution(docp, nlp_oc_sol)

        # plot the OptimalControl solution
        plt = plot(
            ocp_sol;
            state_style=(color=1,),
            costate_style=(color=1, legend=:none),
            control_style=(color=1, legend=:none),
            path_style=(color=1, legend=:none),
            dual_style=(color=1, legend=:none),
            size=(816, 220*(n+m)),
            label="OptimalControl",
            leftmargin=20mm,
        )
        for i in 2:n
            plot!(plt[i]; legend=:none)
        end

        # import JuMP model
        nlp_jp = eval(problem)(JuMPBackend())

        # solve
        set_optimizer(nlp_jp, Ipopt.Optimizer)
        set_optimizer_attribute(nlp_jp, "max_iter", 0)
        optimize!(nlp_jp)

        # plot
        t = time_grid(problem, nlp_jp)     # t0, ..., tN = tf
        x = state(problem, nlp_jp)         # function of time
        u = control(problem, nlp_jp)       # function of time
        p = costate(problem, nlp_jp)       # function of time

        for i in 1:n # state
            label = i == 1 ? "JuMP" : :none
            plot!(plt[i], t, t -> x(t)[i]; color=2, linestyle=:dash, label=label)
        end

        for i in 1:n # costate
            plot!(plt[n+i], t, t -> -p(t)[i]; color=2, linestyle=:dash, label=:none)
        end

        for i in 1:m # control
            plot!(plt[2n+i], t, t -> u(t)[i]; color=2, linestyle=:dash, label=:none)
        end

        return plt
    end
    nothing # hide
    ```

    ```@raw html
    </details>
    </br>
    ```

    ```@example main
    plot_initial_guess(:$PROBLEM)
    ```

    ## Solve the problem

    ### OptimalControl model

    Import the OptimalControl model and solve it.

    ```@example main
    # import DOCP model
    docp = $PROBLEM(OptimalControlBackend())

    # get NLP model
    nlp_oc = nlp_model(docp)

    # solve
    nlp_oc_sol = NLPModelsIpopt.ipopt(
        nlp_oc;
        print_level=4,
        tol=1e-8,
        mu_strategy="adaptive",
        sb="yes",
    )
    nothing # hide
    ```

    The problem has the following numbers of steps, variables and constraints.

    ```@example main
    push!(data_pb,
        (
            Problem=:$PROBLEM,
            Grid_Size=metadata[:$PROBLEM][:N],
            Variables=get_nvar(nlp_oc),
            Constraints=get_ncon(nlp_oc),
        )
    )
    data_pb # hide
    ```

    ### JuMP model

    Import the JuMP model and solve it.

    ```@example main
    # import model
    nlp_jp = $PROBLEM(JuMPBackend())

    # solve
    set_optimizer(nlp_jp, Ipopt.Optimizer)
    set_optimizer_attribute(nlp_jp, "print_level", 4)
    set_optimizer_attribute(nlp_jp, "tol", 1e-8)
    set_optimizer_attribute(nlp_jp, "mu_strategy", "adaptive")
    set_optimizer_attribute(nlp_jp, "linear_solver", "mumps")
    set_optimizer_attribute(nlp_jp, "sb", "yes")
    optimize!(nlp_jp)
    ```

    ## Numerical comparisons

    Let's get the flag, the number of iterations and the objective value from the resolutions.

    ```@example main
    # from OptimalControl model
    push!(data_re,
        (
            Model=:OptimalControl,
            Flag=nlp_oc_sol.status,
            Iterations=nlp_oc_sol.iter,
            Objective=nlp_oc_sol.objective,
        )
    )

    # from JuMP model
    push!(data_re,
        (
            Model=:JuMP,
            Flag=termination_status(nlp_jp),
            Iterations=barrier_iterations(nlp_jp),
            Objective=objective_value(nlp_jp),
        )
    )
    data_re # hide
    ```    

    We compare the OptimalControl and JuMP solutions in terms of the number of iterations, the \$L^2\$-norm of the differences in the state, control, and variable, as well as the objective values. Both absolute and relative errors are reported.

    ```@raw html
    <details><summary>Click to unfold and get the code of the numerical comparison.</summary>
    ```

    ```@example main
    function L2_norm(T, X)
        # T and X are supposed to be one dimensional
        s = 0.0
        for i in 1:(length(T) - 1)
            s += 0.5 * (X[i]^2 + X[i + 1]^2) * (T[i + 1]-T[i])
        end
        return √(s)
    end

    function numerical_comparison(problem, docp, nlp_oc_sol, nlp_jp)

        # get relevant data from OptimalControl model
        ocp_sol = build_ocp_solution(docp, nlp_oc_sol) # build an ocp solution
        t_oc = time_grid(ocp_sol)
        x_oc = state(ocp_sol).(t_oc)
        u_oc = control(ocp_sol).(t_oc)
        v_oc = variable(ocp_sol)
        o_oc = objective(ocp_sol)
        i_oc = iterations(ocp_sol)

        # get relevant data from JuMP model
        t_jp = time_grid(problem, nlp_jp)
        x_jp = state(problem, nlp_jp).(t_jp)
        u_jp = control(problem, nlp_jp).(t_jp)
        o_jp = objective(problem, nlp_jp)
        v_jp = variable(problem, nlp_jp)
        i_jp = iterations(problem, nlp_jp)

        x_vars = metadata[problem][:state_name]
        u_vars = metadata[problem][:control_name]
        v_vars = metadata[problem][:variable_name]

        println("┌─ ", string(problem))
        println("│")

        # number of iterations
        println("├─  Number of iterations")
        println("│")
        println("│     OptimalControl : ", i_oc)
        println("│     JuMP           : ", i_jp)
        println("│")

        # state
        for i in eachindex(x_vars)
            xi_oc = [x_oc[k][i] for k in eachindex(t_oc)]
            xi_jp = [x_jp[k][i] for k in eachindex(t_jp)]
            L2_oc = L2_norm(t_oc, xi_oc)
            L2_jp = L2_norm(t_oc, xi_jp)
            L2_ae = L2_norm(t_oc, xi_oc-xi_jp)
            L2_re = L2_ae/(0.5*(L2_oc + L2_jp))

            println("├─  State \$(x_vars[i]) (L2 norm)")
            println("│")
            #println("│     OptimalControl : ", L2_oc)
            #println("│     JuMP           : ", L2_jp)
            println("│     Absolute error : ", L2_ae)
            println("│     Relative error : ", L2_re)
            println("│")
        end

        # control
        for i in eachindex(u_vars)
            ui_oc = [u_oc[k][i] for k in eachindex(t_oc)]
            ui_jp = [u_jp[k][i] for k in eachindex(t_jp)]
            L2_oc = L2_norm(t_oc, ui_oc)
            L2_jp = L2_norm(t_oc, ui_jp)
            L2_ae = L2_norm(t_oc, ui_oc-ui_jp)
            L2_re = L2_ae/(0.5*(L2_oc + L2_jp))

            println("├─  Control \$(u_vars[i]) (L2 norm)")
            println("│")
            #println("│     OptimalControl : ", L2_oc)
            #println("│     JuMP           : ", L2_jp)
            println("│     Absolute error : ", L2_ae)
            println("│     Relative error : ", L2_re)
            println("│")
        end

        # variable
        if !isnothing(v_vars)
            for i in eachindex(v_vars)
                vi_oc = v_oc[i]
                vi_jp = v_jp[i]
                vi_ae = abs(vi_oc-vi_jp)
                vi_re = vi_ae/(0.5*(abs(vi_oc) + abs(vi_jp)))

                println("├─  Variable \$(v_vars[i])")
                println("│")
                #println("│     OptimalControl : ", vi_oc)
                #println("│     JuMP           : ", vi_jp)
                println("│     Absolute error : ", vi_ae)
                println("│     Relative error : ", vi_re)
                println("│")
            end
        end

        # objective
        o_ae = abs(o_oc-o_jp)
        o_re = o_ae/(0.5*(abs(o_oc) + abs(o_jp)))

        println("├─  objective")
        println("│")
        #println("│     OptimalControl : ", o_oc)
        #println("│     JuMP           : ", o_jp)
        println("│     Absolute error : ", o_ae)
        println("│     Relative error : ", o_re)
        println("│")
        println("└─")

        return nothing
    end
    nothing # hide
    ```

    ```@raw html
    </details>
    </br>
    ```

    ```@example main
    numerical_comparison(:$PROBLEM, docp, nlp_oc_sol, nlp_jp)
    ```

    ## Plot the solutions

    Visualise states, costates, and controls from the OptimalControl and JuMP solutions:

    ```@example main
    # build an ocp solution to use the plot from OptimalControl package
    ocp_sol = build_ocp_solution(docp, nlp_oc_sol)

    # dimensions
    n = state_dimension(ocp_sol)   # or length(metadata[:$PROBLEM][:state_name])
    m = control_dimension(ocp_sol) # or length(metadata[:$PROBLEM][:control_name])

    # from OptimalControl solution
    plt = plot(
        ocp_sol;
        state_style=(color=1,),
        costate_style=(color=1, legend=:none),
        control_style=(color=1, legend=:none),
        path_style=(color=1, legend=:none),
        dual_style=(color=1, legend=:none),
        size=(816, 240*(n+m)),
        label="OptimalControl",
        leftmargin=20mm,
    )
    for i in 2:n
        plot!(plt[i]; legend=:none)
    end
    
    # from JuMP solution
    t = time_grid(:$PROBLEM, nlp_jp)     # t0, ..., tN = tf
    x = state(:$PROBLEM, nlp_jp)         # function of time
    u = control(:$PROBLEM, nlp_jp)       # function of time
    p = costate(:$PROBLEM, nlp_jp)       # function of time

    for i in 1:n # state
        label = i == 1 ? "JuMP" : :none
        plot!(plt[i], t, t -> x(t)[i]; color=2, linestyle=:dash, label=label)
    end

    for i in 1:n # costate
        plot!(plt[n+i], t, t -> -p(t)[i]; color=2, linestyle=:dash, label=:none)
    end

    for i in 1:m # control
        plot!(plt[2n+i], t, t -> u(t)[i]; color=2, linestyle=:dash, label=:none)
    end
    plt # hide
    ```
    """

    return documentation
end

function generate_documentation_problems(;
    draft::Union{Bool,Nothing}=nothing, exclude_from_draft::Vector{Symbol}=Symbol[]
)

    # List of problems
    problems_list = problems()

    # 
    problems_pages = []
    for problem in problems_list
        push!(problems_pages, joinpath("problems", string(problem) * ".md"))
    end

    # remove and create problems directory
    rm(joinpath(@__DIR__, "src", "problems"); recursive=true, force=true)
    mkpath(joinpath(@__DIR__, "src", "problems"))
    mkpath(joinpath(@__DIR__, "src", "problems", "assets"))

    # create file for documentation
    for problem in problems_list

        # create the file
        filename = joinpath(@__DIR__, "src", "problems", string(problem) * ".md")
        touch(filename)

        # get the description
        description = read(
            joinpath(@__DIR__, "..", "ext", "Descriptions", string(problem) * ".md"), String
        )

        # generate the content
        draft_problem = problem ∈ exclude_from_draft ? false : draft
        contents = generate_documentation(string(problem), description; draft=draft_problem)

        # write the content in the file
        open(filename, "a") do io
            write(io, contents)
        end
    end

    return problems_pages
end
