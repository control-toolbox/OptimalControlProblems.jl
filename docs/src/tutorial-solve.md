# [Solve a problem](@id solve-problem)

We consider the **Beam** problem. Let's import the OptimalControlProblems package.

```@example main
using OptimalControlProblems
```

## Solving from OptimalControl model

Let's import the problem.

```@example main
using OptimalControl
docp, model = beam(OptimalControlBackend())
nothing # hide
```

The model represents the nonlinear programming problem (NLP) obtained after the discretisation of the optimal control problem (OCP), see the [Introduction](@ref problems-introduction) page. The model is a [`ADNLPModels.ADNLPModel`](@extref), it provides automatic differentiation (AD)-based models that follows the [NLPModels.jl](https://github.com/JuliaSmoothOptimizers/NLPModels.jl) API. 

Then, we can solve the problem using for instance [`NLPModelsIpopt.ipopt`](@extref).

```@example main
using NLPModelsIpopt

# Solve the model
sol = NLPModelsIpopt.ipopt(
    model;
    print_level=5,
    tol=1e-8,
    mu_strategy="adaptive",
    sb="yes",
)
nothing # hide
```

To get the number of iterations:
```@example main
sol.iter
```

In order to recover the state, the control and the costate, we advice to build an optimal control solution and then use the associated getters:

```@example main
ocp_sol = build_OCP_solution(
    docp;
    primal=sol.solution,
    dual=sol.multipliers,
    docp_solution=sol,
)

t = time_grid(ocp_sol)  # t0, ..., tN = tf
x = state(ocp_sol)      # function of time
u = control(ocp_sol)    # function of time
p = costate(ocp_sol)    # function of time
o = objective(ocp_sol)  # Float number

tf = t[end]
println("tf = ", tf)
println("x(tf) = ", x(tf))
println("u(tf) = ", u(tf))
println("p(tf) = ", p(tf))
println("objective value: ", o)
```

!!! note

    If there are some variables to optimise, for instance the final time, then you can get them with the command:

    ```julia
    v = variable(ocp_sol)
    ```

From `ocp_sol` you can plot the state, control and costate trajectories. For more details about the `plot` method for optimal control problems, please visit the [Plot Manual](@extref OptimalControl manual-plot).

```@example main
using Plots
plt = plot(ocp_sol; color=1, size=(800, 700), control_style=(label="OptimalControl", ))
```

## Solving from JuMP model

Lest's import the JuMP model.

```@example main
using JuMP
model = beam(JuMPBackend())
```

Then, we can solve the problem using the [`JuMP.optimize!`](@extref) function.	

```@example main
using Ipopt

# Set the optimizer
set_optimizer(model, Ipopt.Optimizer)

# Set the optimizer attributes
set_optimizer_attribute(model, "tol", 1e-8)
set_optimizer_attribute(model, "mu_strategy", "adaptive")
set_optimizer_attribute(model, "linear_solver", "mumps")
set_optimizer_attribute(model, "sb", "yes")

# Solve the model
optimize!(model)
```

To get the number of iterations:

```@example main
barrier_iterations(model)
```

To get the objective value:

```@example main
objective_value(model)
```

To get the time grid, the state, the control and the costate, OptimalControlProblems provides the following getters:

```@example main
problem = :beam

t = time_grid(problem, model)    # t0, ..., tN = tf
x = state(problem, model)        # function of time
u = control(problem, model)      # function of time
p = costate(problem, model)      # function of time

tf = t[end]
println("tf = ", tf)
println("x(tf) = ", x(tf))
println("u(tf) = ", u(tf))
println("p(tf) = ", p(tf))
```

We can add the state, costate and control to the plot.

```@example main
n = length(OptimalControlProblems.metadata[problem][:state_name]) # dimension of the state
m = length(OptimalControlProblems.metadata[problem][:control_name]) # dimension of the control

for i in 1:n # state
    plot!(plt[i], t, t -> x(t)[i]; color=2, linestyle=:dash, label=:none)
end

for i in 1:n # costate
    plot!(plt[n+i], t, t -> p(t)[i]; color=2, linestyle=:dash, label=:none)
end

for i in 1:m # control
    plot!(plt[2n+i], t, t -> u(t)[i]; color=2, linestyle=:dash, label="JuMP")
end
plt # hide
```