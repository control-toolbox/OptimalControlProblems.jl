# [Get a problem](@id get-problem)

Each problem in OptimalControlProblems package is modelled in JuMP and OptimalControl. To get a model, we need to indicate either JuMP or OptimalControl backend.

## Get an OptimalControl model

### NLP Model

To get an OptimalControl model, first [install OptimalControl](https://control-toolbox.org/OptimalControl.jl/stable/#Installation) and import the packages:

```@example main_oc
using OptimalControl
using OptimalControlProblems
```

Then, to get the OptimalControl model of the beam problem, execute:

```@example main_oc
_, model = beam(OptimalControlBackend())
model # hide
```

The model represents the nonlinear programming problem (NLP) obtained after the discretisation of the optimal control problem (OCP), see the [Introduction](@ref problems-introduction) page. The model is a [`ADNLPModels.ADNLPModel`](@extref), it provides automatic differentiation (AD)-based models that follows the [NLPModels.jl](https://github.com/JuliaSmoothOptimizers/NLPModels.jl) API.

### DOCP

You have also access to the DOCP model, which corresponds to the discretised optimal control problem. For further details, we refer to the following [tutorial](@extref Tutorials Discretization-and-NLP-problem) or the documentation of [`CTDirect.DOCP`](@extref).

```@example main_oc
docp, model = beam(OptimalControlBackend())
nothing # hide
```

### Number of variables, constraints and nonzeros

The model follows the [NLPModels.jl](https://github.com/JuliaSmoothOptimizers/NLPModels.jl) API. See the existing [Attributes](https://jso.dev/NLPModels.jl/stable/#Attributes) and the getters (`get_X` functions) [here](https://jso.dev/NLPModels.jl/stable/reference). To get the number of variables, use:

```@example main_oc
using NLPModels
get_nvar(model)
```

To get the number of constraints:

```@example main_oc
get_ncon(model)
```

To get the number of nonzeros:

```@example main_oc
nnzo = get_nnzo(model) # Gradient of the Objective
nnzj = get_nnzj(model) # Jacobian of the constraints
nnzh = get_nnzh(model) # Hessian of the Lagrangian

println("nnzo = ", nnzo)
println("nnzj = ", nnzj)
println("nnzh = ", nnzh)
```

!!! note

    You can also evaluate the objective function, the constraints... See the [API page](https://jso.dev/NLPModels.jl/stable/api).

### Number of steps

The number of steps $N$ is given by the metadata:

```@example main_oc
OptimalControlProblems.metadata[:beam][:N]
```

!!! note

    The grid $\{t_0, \ldots, t_N=t_f\}$ is of length $N+1$.

Each problem is parameterised by the number of steps:

```@example main_oc
docp, model = beam(OptimalControlBackend(); N=100)
get_nvar(model)
```

```@example main_oc
docp, model = beam(OptimalControlBackend(); N=200)
get_nvar(model)
```

## Get a JuMP model

To get a JuMP model, first [install JuMP](https://jump.dev/JuMP.jl/stable/installation/#Installation-Guide) and import the packages:

```@example main_jp
using JuMP
using OptimalControlProblems
```

Then, to get the JuMP model of the beam problem, execute:

```@example main_jp
model = beam(JuMPBackend())
```

!!! note

    To interact with the JuMP model, we refer to the [JuMP documentation](https://jump.dev/JuMP.jl).