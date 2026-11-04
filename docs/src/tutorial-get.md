# [Get a problem](@id get-problem)

Each problem in the **OptimalControlProblems** package is modelled in JuMP, in OptimalControl, or in both. To obtain a model, you need to specify either the JuMP or the OptimalControl backend, but first import the package:

```@example main_oc
using OptimalControlProblems
```

```@setup main_jp
using OptimalControlProblems
```

## Get an OptimalControl model

To get an OptimalControl model, first [install](https://control-toolbox.org/OptimalControl.jl/dev/#Installation) OptimalControl and import the package:

```@example main_oc
using OptimalControl
```

Then, to obtain the beam problem, run:

```@example main_oc
pb = beam(OptimalControlBackend())
```

The returned [`Problem`](@ref OptimalControlProblems.Problem) holds the **continuous** optimal control problem (OCP) and its initial guess; it is not discretised. Read it through its accessors:

```@example main_oc
name(pb)
```

```@example main_oc
initial_guess(pb)
```

```@example main_oc
ocp(pb)
```

The scheme, the grid, the modeler and the solver are chosen when solving, see [Solve a problem](@ref solve-problem). You can replace any default parameter value:

```@example main_oc
pb = beam(OptimalControlBackend(); parameters=(tf=2, ))
nothing # hide
```

To get the list of `:beam` parameters and the default values, make:

```@example main_oc
problem_metadata(:beam)[:parameters]
```

The metadata of a built problem holds the parameters it was built with:

```@example main_oc
metadata(pb)[:parameters]
```

To have the description of the Beam problem parameters, either check the Beam [page](@ref description-beam) or the [code](https://github.com/control-toolbox/OptimalControlProblems.jl/blob/main/ext/OptimalControlModels/beam.jl).

### Number of variables, constraints, and nonzeros

To get the nonlinear programming problem (NLP), discretise the OCP and build the NLP model with a modeler, here [`ADNLPModels.ADNLPModel`](@extref):

```@example main_oc
pb = beam(OptimalControlBackend())
docp = discretize(ocp(pb), OptimalControl.Collocation(grid_size=100, scheme=:trapeze))
init = build_initial_guess(ocp(pb), initial_guess(pb))
nlp = nlp_model(docp, init, OptimalControl.ADNLP())
nothing # hide
```

The `nlp` model follows the [NLPModels.jl](https://github.com/JuliaSmoothOptimizers/NLPModels.jl) API. See the existing [Attributes](https://jso.dev/NLPModels.jl/stable/#Attributes) and the available [getter functions](https://jso.dev/NLPModels.jl/stable/reference) (`get_X`).

To get the number of variables, import the package:

```@example main_oc
using NLPModels
```

and then, use the associated getter:

```@example main_oc
get_nvar(nlp)
```

To get the number of constraints:

```@example main_oc
get_ncon(nlp)
```

To get the number of nonzeros:

```@example main_oc
nnzo = get_nnzo(nlp) # Gradient of the objective
nnzj = get_nnzj(nlp) # Jacobian of the constraints
nnzh = get_nnzh(nlp) # Hessian of the Lagrangian

println("nnzo = ", nnzo)
println("nnzj = ", nnzj)
println("nnzh = ", nnzh)
```

!!! note
    You can also evaluate the objective function, the constraints, and more. See the [API page](https://jso.dev/NLPModels.jl/stable/api).

### Number of steps

The default number of steps $N$ is stored in the metadata:

```@example main_oc
N = problem_metadata(:beam)[:grid_size]
```

!!! note
    The grid $\{t_0, \ldots, t_N = t_f\}$ has length $N+1$.

The number of steps is chosen when discretising:

```@example main_oc
docp = discretize(ocp(pb), OptimalControl.Collocation(grid_size=200, scheme=:trapeze))
get_nvar(nlp_model(docp, init, OptimalControl.ADNLP()))
```

## Get a JuMP model

To get a JuMP model, [install](https://jump.dev/JuMP.jl/stable/installation/#Installation-Guide) JuMP and import the package:

```@example main_jp
using JuMP
```

Then, to obtain the JuMP model of the beam problem, run:

```@example main_jp
nlp = beam(JuMPBackend())
```

For details on how to interact with the JuMP model, see the [JuMP documentation](https://jump.dev/JuMP.jl). In particular, you can pass any arguments and keyword arguments of [`JuMP.Model`](@extref) to the `beam` problem or any other.

```julia
using Ipopt
nlp = beam(JuMPBackend(), Ipopt.Optimizer; add_bridges=true)
```

!!! note

    - As with `OptimalControlBackend`, you can replace any default parameter value.
    - You can transform the JuMP model into a [`NLPModelsJuMP.MathOptNLPModel`](@extref) and then use all the API of [NLPModels.jl](https://github.com/JuliaSmoothOptimizers/NLPModels.jl). See this [tutorial](https://jso.dev/NLPModelsJuMP.jl/dev/tutorial) for more details.