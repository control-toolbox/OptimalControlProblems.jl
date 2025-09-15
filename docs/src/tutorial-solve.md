# [Solve a problem](@id solve-problem)

In this tutorial, we consider the Beam problem. First, import the OptimalControlProblems package to access the problem:

```@example main
using OptimalControlProblems
```

## Solving from an OptimalControl model

First, import the problem:

```@example main
using OptimalControl
docp = beam(OptimalControlBackend())
nlp = nlp_model(docp)
nothing # hide
```

The `nlp` model represents the nonlinear programming problem (NLP) obtained after discretising the optimal control problem (OCP). See the [Introduction](@ref problems-introduction) page for details. The model is an [`ADNLPModels.ADNLPModel`](@extref), which provides automatic differentiation (AD)-based models that follow the [NLPModels.jl](https://github.com/JuliaSmoothOptimizers/NLPModels.jl) API. 

We can then solve the problem using, for instance, [`NLPModelsIpopt.ipopt`](@extref):

```@example main
using NLPModelsIpopt

# Solve the model
nlp_sol = NLPModelsIpopt.ipopt(
    nlp;
    print_level=5,
    tol=1e-8,
    mu_strategy="adaptive",
)
nothing # hide
```

To get the number of iterations from the NLP solution:

```@example main
nlp_sol.iter
```

and the objective value:

```@example main
nlp_sol.objective
```

To recover the state, control, and costate, we recommend building an optimal control solution and using the associated getters (you can also retrieve the number of iterations and the objective value from the OCP solution):

```@example main
ocp_sol = build_ocp_solution(docp, nlp_sol)

t = time_grid(ocp_sol)  # t0, ..., tN = tf
x = state(ocp_sol)      # function of time
u = control(ocp_sol)    # function of time
p = costate(ocp_sol)    # function of time
o = objective(ocp_sol)  # scalar objective value
i = iterations(ocp_sol) # number of iteration

tf = t[end]
println("tf = ", tf)
println("x(tf) = ", x(tf))
println("u(tf) = ", u(tf))
println("p(tf) = ", p(tf))
println("objective:  ", o)
println("iterations: ", i)
```

!!! note
    If the problem includes additional optimisation variables, such as the final time, you can retrieve them with:

    ```julia
    v = variable(ocp_sol)
    ```

From `ocp_sol` you can also plot the state, control, and costate trajectories. For more details about plotting optimal control problems, see the [Plot Manual](@extref OptimalControl manual-plot).

```@example main
using Plots
plt = plot(ocp_sol; color=1, size=(800, 700), control_style=(label="OptimalControl", ))
```

## Solving from a JuMP model

First, import the JuMP model:

```@example main
using JuMP
nlp = beam(JuMPBackend())
```

We can then solve the problem using the [`JuMP.optimize!`](@extref) function:

```@example main
using Ipopt

# Set the optimiser
set_optimizer(nlp, Ipopt.Optimizer)

# Set optimiser attributes
set_optimizer_attribute(nlp, "tol", 1e-8)
set_optimizer_attribute(nlp, "mu_strategy", "adaptive")
set_optimizer_attribute(nlp, "linear_solver", "mumps")

# Solve the model
optimize!(nlp)
```

To get the number of iterations:

```@example main
barrier_iterations(nlp)
```

To get the objective value:

```@example main
objective_value(nlp)
```

To get the time grid, state, control, and costate, but also to retrieve the number of iterations and the objective value, OptimalControlProblems provides the following getters:

```@example main
t = time_grid(:beam, nlp)    # t0, ..., tN = tf
x = state(:beam, nlp)        # function of time
u = control(:beam, nlp)      # function of time
p = costate(:beam, nlp)      # function of time
o = objective(:beam, nlp)    # scalar objective value
i = iterations(:beam, nlp)   # number of iteration

tf = t[end]
println("tf = ", tf)
println("x(tf) = ", x(tf))
println("u(tf) = ", u(tf))
println("p(tf) = ", p(tf))
println("objective:  ", o)
println("iterations: ", i)
```

!!! note
    If the `problem` includes additional optimisation variables, such as the final time, you can retrieve them with:

    ```julia
    v = variable(problem, nlp)
    ```

Now, we can add the state, costate, and control to the plot:

```@example main
n = length(metadata(:beam)[:state_components])   # dimension of the state
m = length(metadata(:beam)[:control_components]) # dimension of the control

for i in 1:n # state
    plot!(plt[i], t, t -> x(t)[i]; color=2, linestyle=:dash, label=:none)
end

for i in 1:n # costate
    plot!(plt[n+i], t, t -> -p(t)[i]; color=2, linestyle=:dash, label=:none)
end

for i in 1:m # control
    plot!(plt[2n+i], t, t -> u(t)[i]; color=2, linestyle=:dash, label="JuMP")
end
plt # hide
```

!!! note

    The costate from JuMP is the opposite of the costate from OptimalControl, that is why we plot $-p(t)$.