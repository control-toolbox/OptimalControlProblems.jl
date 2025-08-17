function generate_documentation(PROBLEM::String; draft::Union{Bool,Nothing})

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

We consider the `:$PROBLEM` problem. 

## Packages

First, import all the necessary packages and define the DataFrames to store the data from the model and the resolutions.

```@example main
using OptimalControlProblems    # to access the Beam model
using OptimalControl            # to import the OptimalControl model
using NLPModelsIpopt            # to solve the model with Ipopt
using DataFrames                # to store data
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

## OptimalControl model

### Solve the problem

Import the problem and solve it.

```@example main
# import model
docp, model_oc = $PROBLEM(OptimalControlBackend())

# solve
nlp_sol = NLPModelsIpopt.ipopt(
    model_oc;
    print_level=4,
    tol=1e-8,
    mu_strategy="adaptive",
    sb="yes",
)

# build an optimal control solution
ocp_sol = build_OCP_solution(docp; primal=nlp_sol.solution, dual=nlp_sol.multipliers, docp_solution=nlp_sol)
nothing # hide
```

For numerical comparison with the JuMP model resolution, define:

```@example main
t_oc = time_grid(ocp_sol)
x_oc = state(ocp_sol).(t_oc)
u_oc = control(ocp_sol).(t_oc)
o_oc = objective(ocp_sol)
v_oc = variable(ocp_sol)
i_oc = nlp_sol.iter
nothing # hide
```

### Store and print the data

Store the following data about the problem:

```@example main
push!(data_pb,
    (
        Problem=:$PROBLEM,
        Grid_Size=OptimalControlProblems.metadata[:$PROBLEM][:N],
        Variables=get_nvar(model_oc),
        Constraints=get_ncon(model_oc),
    )
)
```

And store the following data about the resolution:

```@example main
push!(data_re,
    (
        Model=:OptimalControl,
        Flag=nlp_sol.status,
        Iterations=nlp_sol.iter,
        Objective=objective(ocp_sol),
    )
)
```

### Plot the solution

To plot the solution, get the number of states and controls from the metadata:

```@example main
x_vars = OptimalControlProblems.metadata[:$PROBLEM][:state_name]
u_vars = OptimalControlProblems.metadata[:$PROBLEM][:control_name]

n = length(x_vars) # number of states
m = length(u_vars) # number of controls

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
plt # hide
```

## JuMP model

### Solve the problem

Import the JuMP model and solve it.

```@example main
# import model
model_jp = $PROBLEM(JuMPBackend())

# solve
set_optimizer(model_jp, Ipopt.Optimizer)
set_optimizer_attribute(model_jp, "print_level", 4)
set_optimizer_attribute(model_jp, "tol", 1e-8)
set_optimizer_attribute(model_jp, "mu_strategy", "adaptive")
set_optimizer_attribute(model_jp, "linear_solver", "mumps")
set_optimizer_attribute(model_jp, "sb", "yes")
optimize!(model_jp)
```

For the numerical comparison, define:

```@example main
t_jp = time_grid(:$PROBLEM, model_jp)
x_jp = state(:$PROBLEM, model_jp).(t_jp)
u_jp = control(:$PROBLEM, model_jp).(t_jp)
o_jp = objective_value(model_jp)
v_jp = variable(:$PROBLEM, model_jp)
i_jp = barrier_iterations(model_jp)
nothing # hide
```

### Store and print the data

Store the results of the resolution:

```@example main
push!(data_re,
    (
        Model=:JuMP,
        Flag=termination_status(model_jp),
        Iterations=barrier_iterations(model_jp),
        Objective=objective_value(model_jp),
    )
)
```

### Plot the solution

Add the state, costate, and control from the JuMP model to the plot:

```@example main
t = time_grid(:$PROBLEM, model_jp)     # t0, ..., tN = tf
x = state(:$PROBLEM, model_jp)         # function of time
u = control(:$PROBLEM, model_jp)       # function of time
p = costate(:$PROBLEM, model_jp)       # function of time

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

## Initial guess

The initial guess (or first iterate) is obtained by fixing `max_iter=0` in the solver:

```@raw html
<details><summary>Unfold to see the code for plotting the initial guess.</summary>
```

```@example main
function plot_initial_guess()

    # import OptimalControl model
    docp, model_oc = $PROBLEM(OptimalControlBackend())

    # solve
    nlp_sol = NLPModelsIpopt.ipopt(
        model_oc;
        max_iter=0,
        print_level=5,
        tol=1e-8,
        mu_strategy="adaptive",
        sb="yes",
    )

    # build an optimal control solution
    ocp_sol = build_OCP_solution(docp; primal=nlp_sol.solution, dual=nlp_sol.multipliers, docp_solution=nlp_sol)

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
    model_jp = $PROBLEM(JuMPBackend())

    # solve
    set_optimizer(model_jp, Ipopt.Optimizer)
    set_optimizer_attribute(model_jp, "max_iter", 0)
    set_optimizer_attribute(model_jp, "print_level", 5)
    set_optimizer_attribute(model_jp, "tol", 1e-8)
    set_optimizer_attribute(model_jp, "mu_strategy", "adaptive")
    set_optimizer_attribute(model_jp, "linear_solver", "mumps")
    set_optimizer_attribute(model_jp, "sb", "yes")
    optimize!(model_jp)

    # plot
    t = time_grid(:$PROBLEM, model_jp)     # t0, ..., tN = tf
    x = state(:$PROBLEM, model_jp)         # function of time
    u = control(:$PROBLEM, model_jp)       # function of time
    p = costate(:$PROBLEM, model_jp)       # function of time

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
plot_initial_guess()
```

## Numerical comparison

Next, compare the number of iterations required to obtain the solutions. Also compare the final times, objective values, and the state and costate trajectories in L² norm.

```@raw html
<details><summary>Unfold to get the code of the numerical comparison.</summary>
```

```@example main
v_vars = OptimalControlProblems.metadata[:$PROBLEM][:variable_name]

function L2_norm(T, X)
    # T and X are supposed to be one dimensional
    s = 0.0
    for i in 1:(length(T) - 1)
        s += 0.5 * (X[i]^2 + X[i + 1]^2) * (T[i + 1]-T[i])
    end
    return √(s)
end

function numerical_comparison()

    println("┌─ ", "$PROBLEM")
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
        println("│     OptimalControl : ", L2_oc)
        println("│     JuMP           : ", L2_jp)
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
        println("│     OptimalControl : ", L2_oc)
        println("│     JuMP           : ", L2_jp)
        println("│     Absolute error : ", L2_ae)
        println("│     Relative error : ", L2_re)
        println("│")
    end

    # control
    if !isnothing(v_vars)
        for i in eachindex(v_vars)
            vi_oc = v_oc[i]
            vi_jp = v_jp[i]
            vi_ae = abs(vi_oc-vi_jp)
            vi_re = vi_ae/(0.5*(abs(vi_oc) + abs(vi_jp)))

            println("├─  Variable \$(v_vars[i])")
            println("│")
            println("│     OptimalControl : ", vi_oc)
            println("│     JuMP           : ", vi_jp)
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
    println("│     OptimalControl : ", o_oc)
    println("│     JuMP           : ", o_jp)
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
numerical_comparison()
```
"""

    return documentation

end

function generate_documentation_problems(; draft::Union{Bool,Nothing}=nothing, exclude_from_draft::Vector{Symbol}=Symbol[])

    # List of problems
    problems = available_problems()

    # 
    problems_pages = []
    for problem in problems
       push!(problems_pages, joinpath("problems", string(problem) * ".md"),) 
    end

    # remove and create problems directory
    rm(joinpath(@__DIR__, "src", "problems"), recursive=true, force=true)
    mkpath(joinpath(@__DIR__, "src", "problems"))
    mkpath(joinpath(@__DIR__, "src", "problems", "assets"))

    # create file for documentation
    for problem in problems

        println("generating doc for ", problem)

        # create the file
        filename = joinpath(@__DIR__, "src", "problems", string(problem) * ".md")
        touch(filename)

        # generate the content
        draft_problem = problem ∈ exclude_from_draft ? false : draft
        contents = generate_documentation(string(problem); draft=draft_problem)

        # write the content in the file
        open(filename, "a") do io
            write(io, contents)
        end
    end

    return problems_pages

end