# [Solve a problem](@id solve-problem)

In this tutorial, we consider the Beam problem. First, import the OptimalControlProblems package to access the problem:

```@example main
using OptimalControlProblems
```

## Solving from an OptimalControl model

First, get the problem:

```@example main
using OptimalControl
pb = beam(OptimalControlBackend())
nothing # hide
```

The problem `pb` holds the continuous optimal control problem, `model(pb)`, and its initial guess, `initial_guess(pb)`. The discretisation, the modeler and the solver are chosen in `solve`. Here, we discretise with the trapezoidal rule on the default grid of the problem, model the NLP with [ADNLPModels.jl](https://github.com/JuliaSmoothOptimizers/ADNLPModels.jl) and solve it with [Ipopt](https://github.com/coin-or/Ipopt):

```@example main
using NLPModelsIpopt

N = problem_metadata(:beam)[:grid_size]
ocp_sol = solve(
    model(pb), :collocation, :adnlp, :ipopt;
    init=initial_guess(pb),
    grid_size=N,
    scheme=:trapeze,
    tol=1e-8,
    mu_strategy="adaptive",
)
nothing # hide
```

!!! note
    The same choices can be made in explicit mode, with the strategies as arguments; see the [explicit mode](https://control-toolbox.org/OptimalControl.jl/dev/solve/explicit-mode) of OptimalControl:

    ```julia
    ocp_sol = solve(model(pb);
        init=initial_guess(pb),
        discretizer=OptimalControl.Collocation(grid_size=N, scheme=:trapeze),
        modeler=OptimalControl.ADNLP(),
        solver=OptimalControl.Ipopt(tol=1e-8),
    )
    ```

To recover the state, control, and costate, use the getters of the solution:

```@example main
t = time_grid(ocp_sol)  # t0, ..., tN = tf
x = state(ocp_sol)      # function of time
u = control(ocp_sol)    # function of time
p = costate(ocp_sol)    # function of time
o = objective(ocp_sol)  # scalar objective value
i = iterations(ocp_sol) # number of iterations

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

From `ocp_sol` you can also plot the state, control, and costate trajectories. For more details about plotting optimal control problems, see the [Plot](@extref OptimalControl results-plot) page.

```@example main
using Plots
plt = plot(ocp_sol; color=1, size=(700, 600), control_style=(label="OptimalControl", ))
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
t = time_grid(nlp)    # t0, ..., tN = tf
x = state(nlp)        # function of time
u = control(nlp)      # function of time
p = costate(nlp)      # function of time
o = objective(nlp)    # scalar objective value
i = iterations(nlp)   # number of iteration

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
    v = variable(nlp)
    ```

Now, we can add the state, costate, and control to the plot:

```@example main
n = state_dimension(nlp)
m = control_dimension(nlp)

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