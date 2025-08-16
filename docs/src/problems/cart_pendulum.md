# cart_pendulum

We consider the `:cart_pendulum` problem. 

## Packages

Let's import all the necessary packages and define the dataframes to store the data from the model and the resolutions.

```@example main
using OptimalControlProblems    # to get access to the Beam model
using OptimalControl            # to import the OptimalControl model
using NLPModelsIpopt            # to solve the model with ipopt
using DataFrames                # to store data
using NLPModels                 # to retrieve data from NLP solution
using Plots                     # to plot the trajectories
using Plots.PlotMeasures        # for leftmargin, bottommargin
using JuMP                      # to import the JuMP model
using Ipopt                     # to solve the JuMP model with ipopt

data_pb = DataFrame(            # to store data from the problem
    Problem=Symbol[],
    Grid_Size=Int[],
    Variables=Int[],
    Constraints=Int[],
)

data_re = DataFrame(            # to store data from the resolutions
    Model=Symbol[],
    Flag=Any[],
    Iterations=Int[],
    Objective=Float64[],
)
nothing # hide
```

## OptimalControl model

### Solve the problem

Let's import the problem and solve it.

```@example main
# import model
docp, model_oc = cart_pendulum(OptimalControlBackend())

# solve
nlp_sol = NLPModelsIpopt.ipopt(
    model_oc;
    print_level=5,
    tol=1e-8,
    mu_strategy="adaptive",
    sb="yes",
)

# build an optimal control solution
ocp_sol = build_OCP_solution(docp; primal=nlp_sol.solution, dual=nlp_sol.multipliers, docp_solution=nlp_sol)
nothing # hide
```

For a numerical comparison with the JuMP model resolution, we define the following.

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

We store the following data about the problem.

```@example main
push!(data_pb,
    (
        Problem=:cart_pendulum,
        Grid_Size=OptimalControlProblems.metadata[:cart_pendulum][:N],
        Variables=get_nvar(model_oc),
        Constraints=get_ncon(model_oc),
    )
)
```

And we store the following data about the resolution.

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

To plot the solution we need the number of states and controls that we get from the metadata.

```@example main
x_vars = OptimalControlProblems.metadata[:cart_pendulum][:state_name]
u_vars = OptimalControlProblems.metadata[:cart_pendulum][:control_name]

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

Lest's import the JuMP model and solve it.

```@example main
# import model
model_jp = cart_pendulum(JuMPBackend())

# solve
set_optimizer(model_jp, Ipopt.Optimizer)
set_optimizer_attribute(model_jp, "print_level", 5)
set_optimizer_attribute(model_jp, "tol", 1e-8)
set_optimizer_attribute(model_jp, "mu_strategy", "adaptive")
set_optimizer_attribute(model_jp, "linear_solver", "mumps")
set_optimizer_attribute(model_jp, "sb", "yes")
optimize!(model_jp)
```

For the numerical comparison we define the following.

```@example main
t_jp = time_grid(:cart_pendulum, model_jp)
x_jp = state(:cart_pendulum, model_jp).(t_jp)
u_jp = control(:cart_pendulum, model_jp).(t_jp)
o_jp = objective_value(model_jp)
v_jp = variable(:cart_pendulum, model_jp)
i_jp = barrier_iterations(model_jp)
nothing # hide
```

### Store and print the data

We store the results of the resolution.

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

We add the state, costate and control from the JuMP model to the plot.

```@example main
t = time_grid(:cart_pendulum, model_jp)     # t0, ..., tN = tf
x = state(:cart_pendulum, model_jp)         # function of time
u = control(:cart_pendulum, model_jp)       # function of time
p = costate(:cart_pendulum, model_jp)       # function of time

for i in 1:n # state
    label = i == 1 ? "JuMP" : :none
    plot!(plt[i], t, t -> x(t)[i]; color=2, linestyle=:dash, label=label)
end

for i in 1:n # costate
    plot!(plt[n+i], t, t -> p(t)[i]; color=2, linestyle=:dash, label=:none)
end

for i in 1:m # control
    plot!(plt[2n+i], t, t -> u(t)[i]; color=2, linestyle=:dash, label=:none)
end
plt # hide
```

## Initial guess

The initial guess (or first iterate) is simple given fixing `max_iter=0` to the solver. We get the following.

```@raw html
<details><summary>Unfold to get the code to plot the initial guess.</summary>
```

```@example main
function plot_initial_guess()

    # import OptimalControl model
    docp, model_oc = cart_pendulum(OptimalControlBackend())

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
    model_jp = cart_pendulum(JuMPBackend())

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
    t = time_grid(:cart_pendulum, model_jp)     # t0, ..., tN = tf
    x = state(:cart_pendulum, model_jp)         # function of time
    u = control(:cart_pendulum, model_jp)       # function of time
    p = costate(:cart_pendulum, model_jp)       # function of time

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

We compare next the number of iterations to get the solutions. We also compare the final times, the objective values and the state and costate trajectories in L2 norm.

```@raw html
<details><summary>Unfold to get the code of the numerical comparison.</summary>
```

```@example main
v_vars = OptimalControlProblems.metadata[:cart_pendulum][:variable_name]

function L2_norm(T, X)
    # T and X are supposed to be one dimensional
    s = 0.0
    for i in 1:(length(T) - 1)
        s += 0.5 * (X[i]^2 + X[i + 1]^2) * (T[i + 1]-T[i])
    end
    return √(s)
end

function numerical_comparison()

    println("┌─ ", "cart_pendulum")
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

        println("├─  State $(x_vars[i]) (L2 norm)")
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

        println("├─  Control $(u_vars[i]) (L2 norm)")
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

            println("├─  Variable $(v_vars[i])")
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