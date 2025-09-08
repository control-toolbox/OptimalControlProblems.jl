# -----------------------------------
# Helper: draft metadata block
# -----------------------------------
function draft_meta(draft::Union{Bool,Nothing})
    if isnothing(draft)
        return ""
    elseif draft
        return """```@meta\nDraft = true\n```"""
    else
        return """```@meta\nDraft = false\n```"""
    end
end

# -----------------------------------
# Helper: left margin for plots
# -----------------------------------
function get_left_margin(problem::Symbol)
    margins = Dict(:beam => "5mm")
    return get(margins, problem, "20mm")
end

# -----------------------------------
# Generate documentation for a problem
# -----------------------------------
function generate_documentation(PROBLEM::String, DESCRIPTION::String; draft::Union{Bool,Nothing})

    TITLE = uppercasefirst(replace(PROBLEM, "_" => " "))
    DRAFT = draft_meta(draft)
    LEFT_MARGIN = get_left_margin(Symbol(PROBLEM))

    documentation=DRAFT * """
    # $TITLE

    ## Description of the problem

    $DESCRIPTION

    ## Numerical set-up

    In this section, we prepare the numerical environment required to study the problem. We begin by importing the relevant Julia packages and then initialise the data frames   that will store the results of our simulations and computations. These structures provide the foundation for solving the problem and for comparing the different solution strategies in a consistent way.

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
    using Printf                    # to print

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

    Before solving the problem, it is often useful to inspect the initial guess (sometimes called the first iterate). This guess is obtained by running the NLP solver with `max_iter = 0`, which evaluates the problem formulation without performing any optimisation steps.  

    We plot the resulting trajectories for both the OptimalControl and JuMP models. Since both backends represent the same mathematical problem, their initial guesses should coincide, providing a useful consistency check before moving on to the optimised solution.

    !!! note "Code to plot the initial guess"

        ```@raw html
        <details><summary>Click to unfold and see the code for plotting the initial guess.</summary>
        ```

        ```@example main
        function plot_initial_guess(problem)

            # -----------------------------
            # Extract dimensions from metadata
            # -----------------------------
            x_vars = metadata[problem][:state_name]
            u_vars = metadata[problem][:control_name]
            n_states = length(x_vars)
            n_controls = length(u_vars)

            # -----------------------------
            # Build OptimalControl problem
            # -----------------------------
            ocp_model = eval(problem)(OptimalControlBackend())
            nlp_oc = nlp_model(ocp_model)

            # Solve NLP with zero iterations (initial guess)
            nlp_oc_sol = NLPModelsIpopt.ipopt(nlp_oc; max_iter=0)

            # Build OptimalControl solution
            ocp_sol = build_ocp_solution(ocp_model, nlp_oc_sol)

            # -----------------------------
            # Plot OptimalControl solution
            # -----------------------------
            plt = plot(
                ocp_sol;
                state_style=(color=1,),
                costate_style=(color=1, legend=:none),
                control_style=(color=1, legend=:none),
                path_style=(color=1, legend=:none),
                dual_style=(color=1, legend=:none),
                size=(816, 220*(n_states+n_controls)),
                label="OptimalControl",
                leftmargin=$LEFT_MARGIN,
            )

            # Hide legend for additional state plots
            for i in 2:n_states
                plot!(plt[i]; legend=:none)
            end

            # -----------------------------
            # Build JuMP model
            # -----------------------------
            nlp_jp = eval(problem)(JuMPBackend())

            # Solve NLP with zero iterations (initial guess)
            set_optimizer(nlp_jp, Ipopt.Optimizer)
            set_optimizer_attribute(nlp_jp, "max_iter", 0)
            optimize!(nlp_jp)

            # Extract trajectories
            t_grid = time_grid(problem, nlp_jp)
            x_fun = state(problem, nlp_jp)
            u_fun = control(problem, nlp_jp)
            p_fun = costate(problem, nlp_jp)

            # -----------------------------
            # Plot JuMP solution on top
            # -----------------------------
            # States
            for i in 1:n_states
                label = i == 1 ? "JuMP" : :none
                plot!(plt[i], t_grid, t -> x_fun(t)[i]; color=2, linestyle=:dash, label=label)
            end

            # Costates
            for i in 1:n_states
                plot!(plt[n_states+i], t_grid, t -> -p_fun(t)[i]; color=2, linestyle=:dash, label=:none)
            end

            # Controls
            for i in 1:n_controls
                plot!(plt[2*n_states+i], t_grid, t -> u_fun(t)[i]; color=2, linestyle=:dash, label=:none)
            end

            return plt
        end
        nothing # hide
        ```

        ```@raw html
        </details>
        ```

    ```@example main
    plot_initial_guess(:$PROBLEM)
    ```

    ## Solving the problem

    To solve an optimal control problem, we can rely on two complementary formulations: the `OptimalControl` backend, which works directly with the discretised control problem, and the `JuMP` backend, which leverages JuMP’s flexible modelling framework.  

    Both approaches generate equivalent NLPs that can be solved with Ipopt, and comparing them ensures consistency between the two formulations.

    Before solving, we can inspect the discretisation details of the problem. The table below reports the number of grid points, decision variables, and constraints associated with the chosen formulation.  

    ```@example main
    push!(data_pb,
        (
            Problem=:$PROBLEM,
            Grid_Size=metadata[:$PROBLEM][:N],
            Variables=get_nvar(nlp_model($PROBLEM(OptimalControlBackend()))),
            Constraints=get_ncon(nlp_model($PROBLEM(OptimalControlBackend()))),
        )
    )
    data_pb # hide
    ```

    ### OptimalControl model

    We first solve the problem using the `OptimalControl` backend. The process begins by importing the problem definition and constructing the associated nonlinear programming (NLP) model. This NLP is then passed to the Ipopt solver, with standard options for tolerance and barrier parameter strategy.  

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

    ### JuMP model

    We now repeat the procedure using the `JuMP` backend. Here, the problem is reformulated as a JuMP model, which offers a flexible and widely used framework for nonlinear optimisation in Julia. The solver settings are chosen to mirror those used previously, so that the results can be compared on an equal footing. 

    ```@example main
    # import model
    nlp_jp = $PROBLEM(JuMPBackend())

    # solve with Ipopt
    set_optimizer(nlp_jp, Ipopt.Optimizer)
    set_optimizer_attribute(nlp_jp, "print_level", 4)
    set_optimizer_attribute(nlp_jp, "tol", 1e-8)
    set_optimizer_attribute(nlp_jp, "mu_strategy", "adaptive")
    set_optimizer_attribute(nlp_jp, "linear_solver", "mumps")
    set_optimizer_attribute(nlp_jp, "sb", "yes")
    optimize!(nlp_jp)
    ```

    ## Numerical comparisons

    In this section, we examine the results of the problem resolutions. We extract the solver status (flag), the number of iterations, and the objective value for each model. This provides a first overview of how each approach performs and sets the stage for a more detailed comparison of the solution trajectories.

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

    We compare the solutions obtained from the OptimalControl and JuMP models by examining the number of iterations required for convergence, the \$L^2\$-norms of the differences in states, controls, and additional variables, and the corresponding objective values. Both absolute and relative errors are reported, providing a clear quantitative measure of the agreement between the two approaches.

    !!! note "Code to print the numerical comparisons"

        ```@raw html
        <details><summary>Click to unfold and get the code of the numerical comparisons.</summary>
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
        
        function print_numerical_comparisons(problem, docp, nlp_oc_sol, nlp_jp)

            # get relevant data from OptimalControl model
            ocp_sol = build_ocp_solution(docp, nlp_oc_sol)
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
            println("├─  Number of Iterations")
            @printf("│     OptimalControl : %d   JuMP : %d\\n", i_oc, i_jp)

            # States
            println("├─  States (L2 Norms)")
            for i in eachindex(x_vars)
                xi_oc = [x_oc[k][i] for k in eachindex(t_oc)]
                xi_jp = [x_jp[k][i] for k in eachindex(t_jp)]
                L2_ae = L2_norm(t_oc, xi_oc - xi_jp)
                L2_re = L2_ae / (0.5 * (L2_norm(t_oc, xi_oc) + L2_norm(t_oc, xi_jp)))
                @printf("│     %-6s Abs: %.3e   Rel: %.3e\\n", x_vars[i], L2_ae, L2_re)
            end

            # Controls
            println("├─  Controls (L2 Norms)")
            for i in eachindex(u_vars)
                ui_oc = [u_oc[k][i] for k in eachindex(t_oc)]
                ui_jp = [u_jp[k][i] for k in eachindex(t_jp)]
                L2_ae = L2_norm(t_oc, ui_oc - ui_jp)
                L2_re = L2_ae / (0.5 * (L2_norm(t_oc, ui_oc) + L2_norm(t_oc, ui_jp)))
                @printf("│     %-6s Abs: %.3e   Rel: %.3e\\n", u_vars[i], L2_ae, L2_re)
            end

            # Variables
            if !isnothing(v_vars)
                println("├─  Variables")
                for i in eachindex(v_vars)
                    vi_oc = v_oc[i]
                    vi_jp = v_jp[i]
                    vi_ae = abs(vi_oc - vi_jp)
                    vi_re = vi_ae / (0.5 * (abs(vi_oc) + abs(vi_jp)))
                    @printf("│     %-6s Abs: %.3e   Rel: %.3e\\n", v_vars[i], vi_ae, vi_re)
                end
            end

            # Objective
            o_ae = abs(o_oc - o_jp)
            o_re = o_ae / (0.5 * (abs(o_oc) + abs(o_jp)))
            println("├─  Objective")
            @printf("│            Abs: %.3e   Rel: %.3e\\n", o_ae, o_re)
            println("└─")
            return nothing
        end
        nothing # hide
        ```

        ```@raw html
        </details>
        ```

    ```@example main
    print_numerical_comparisons(:$PROBLEM, docp, nlp_oc_sol, nlp_jp)
    ```

    ## Plotting the solutions

    In this section, we visualise the trajectories of the states, costates, and controls obtained from both the OptimalControl and JuMP solutions. The plots provide an intuitive way to compare the two approaches and to observe how the constraints and the optimal control influence the system dynamics.  

    For each variable, the OptimalControl solution is shown in solid lines, while the JuMP solution is overlaid using dashed lines. Since both models represent the same mathematical problem, their trajectories should closely coincide, highlighting the consistency between the two formulations.

    ```@example main
    # build an ocp solution to use the plot from OptimalControl package
    ocp_sol = build_ocp_solution(docp, nlp_oc_sol)

    # dimensions
    n = state_dimension(ocp_sol)   # or length(metadata[:$PROBLEM][:state_name])
    m = control_dimension(ocp_sol) # or length(metadata[:$PROBLEM][:control_name])

    # from OptimalControl solution
    plt = plot(
        ocp_sol;
        color=1,
        size=(816, 240*(n+m)),
        label="OptimalControl",
        leftmargin=$LEFT_MARGIN,
    )
    for i in 2:length(plt)
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

# -----------------------------------
# Generate documentation for all problems
# -----------------------------------
function generate_documentation_problems(; draft::Union{Bool,Nothing}=nothing,
                                         exclude_from_draft::Vector{Symbol}=Symbol[])

    problems_list = problems()
    problems_pages = map(p -> joinpath("problems", string(p) * ".md"), problems_list)

    # reset problems directory
    problems_dir = joinpath(@__DIR__, "src", "problems")
    rm(problems_dir; recursive=true, force=true)
    mkpath(problems_dir)
    mkpath(joinpath(problems_dir, "assets"))

    for problem in problems_list
        description = read(joinpath(@__DIR__, "..", "ext", "Descriptions", string(problem) * ".md"), String)
        draft_problem = problem ∈ exclude_from_draft ? false : draft
        contents = generate_documentation(string(problem), description; draft=draft_problem)

        filename = joinpath(problems_dir, string(problem) * ".md")
        write(filename, contents)
    end

    return problems_pages
end