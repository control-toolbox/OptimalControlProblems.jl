```@meta
Draft = false
```

# [Solve a problem](@id solve-problem)

We consider the **Hanging Chain** problem from [COPS package](https://www.mcs.anl.gov/~more/cops/) as an example. The problem is to find the shape of a chain hanging between two points a and b. The chain is assumed to be a uniform cable with a given length L. The aim is to find the shape of the chain that minimizes the potential energy. 

## Solving from OptimalControl model

We need first to import the needed packages and the problem.

```@example main_oc
using OptimalControlProblems
using OptimalControl

docp, model = chain(OptimalControlBackend())
nothing # hide
```

The model represents the nonlinear programming problem (NLP) obtained after the discretisation of the optimal control problem (OCP), see the [Introduction](@ref problems-introduction) page. The model is a [`ADNLPModels.ADNLPModel`](@extref), it provides automatic differentiation (AD)-based models that follows the [NLPModels.jl](https://github.com/JuliaSmoothOptimizers/NLPModels.jl) API. 

Then, we can solve the problem using for instance [`NLPModelsIpopt.ipopt`](@extref).

```@example main_oc
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
```@example main_oc
sol.iter
```

In order to recover the state, the control and the costate, we advice to build an optimal control solution and then use the associated getters:

```@example main_oc
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

```@example main_oc
using Plots
plot(ocp_sol)
```

## Solving from JuMP model

Lest's import the needed packages and the problem.

```@example main_jp
using OptimalControlProblems
using JuMP

model = chain(JuMPBackend())
```

Then, we can solve the problem using the [`JuMP.optimize!`](@extref) function.	

```@example main_jp
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

```@example main_jp
barrier_iterations(model)
```

To get the objective value:

```@example main_jp
objective_value(model)
```

To get the time grid, the state, the control and the costate, OptimalControlProblems provides the following getters:

```@example main_jp
problem = :chain
t = time_grid(problem, model)    # t0, ..., tN = tf
x = state(problem, model)        # Vector of vectors
u = control(problem, model)      # Vector of scalars (since there is 1 control)
p = costate(problem, model)      # Vector of vectors

tf = t[end]
println("tf = ", tf)
println("x(tf) = ", x[end])
println("u(tf) = ", u[end])
println("p(tf) = ", p[end])
```

