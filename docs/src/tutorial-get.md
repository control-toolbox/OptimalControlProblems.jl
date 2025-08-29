# [Get a problem](@id get-problem)

Each problem in the **OptimalControlProblems** package is modelled both in JuMP and in OptimalControl. To obtain a model, you need to specify either the JuMP or the OptimalControl backend.

## Get an OptimalControl model

### DOCP and NLP Model

To get an OptimalControl model, first install [OptimalControl](https://control-toolbox.org/OptimalControl.jl/stable/#Installation) and import the packages:

```@example main_oc
using OptimalControl
using OptimalControlProblems
```

Then, to obtain the OptimalControl model of the beam problem, run:

```@example main_oc
docp = beam(OptimalControlBackend())
nlp = nlp_model(docp)
```

The `nlp` model represents the nonlinear programming problem (NLP) obtained after discretising the optimal control problem (OCP). See the [Introduction](@ref problems-introduction) page for details. The model is an [`ADNLPModels.ADNLPModel`](@extref), which provides automatic differentiation (AD)-based models that follow the [NLPModels.jl](https://github.com/JuliaSmoothOptimizers/NLPModels.jl) API.

!!! note

    You also have access to the DOCP model, which corresponds to the discretised optimal control problem. Roughly speaking, the DOCP model is the union of the NLP and OCP models. For more details, see this [tutorial](@extref Tutorials Discretization-and-NLP-problem) or the documentation of [`CTDirect.DOCP`](@extref). To get the OCP model:

    ```julia
    ocp = ocp_model(docp)
    ```

!!! note

    You can pass any `description` and `kwargs` of [`CTDirect.direct_transcription`](@extref) to the `beam` problem or any other.

    ```julia
    docp = beam(OptimalControlBackend(), :madnlp; grid_size=100, disc_method=:euler)
    ```

### Number of variables, constraints, and nonzeros

The `nlp` model follows the [NLPModels.jl](https://github.com/JuliaSmoothOptimizers/NLPModels.jl) API. See the existing [Attributes](https://jso.dev/NLPModels.jl/stable/#Attributes) and the available getter functions (`get_X`) [here](https://jso.dev/NLPModels.jl/stable/reference).  

To get the number of variables:

```@example main_oc
using NLPModels
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

The number of steps $N$ is stored in the metadata:

```@example main_oc
metadata[:beam][:N]
```

!!! note
    The grid $\{t_0, \ldots, t_N = t_f\}$ has length $N+1$.

Each problem can be parameterised by the number of steps:

```@example main_oc
docp = beam(OptimalControlBackend(); N=100)
nlp = nlp_model(docp)
get_nvar(nlp)
```

```@example main_oc
docp = beam(OptimalControlBackend(); N=200)
nlp = nlp_model(docp)
get_nvar(nlp)
```

## Get a JuMP model

To get a JuMP model, first [install JuMP](https://jump.dev/JuMP.jl/stable/installation/#Installation-Guide) and import the packages:

```@example main_jp
using JuMP
using OptimalControlProblems
```

Then, to obtain the JuMP model of the beam problem, run:

```@example main_jp
nlp = beam(JuMPBackend())
```

!!! note
    For details on how to interact with the JuMP model, see the [JuMP documentation](https://jump.dev/JuMP.jl). In particular, you can pass any arguments and keyword arguments of [`JuMP.Model`](@extref) to the `beam` problem or any other.

    ```julia
    using Ipopt
    nlp = beam(JuMPBackend(), Ipopt.Optimizer; add_bridges=true)
    ``` 

!!! note
    You can transform a JuMP model into a `MathOptNLPModel` and then use all the API of [NLPModels.jl](https://github.com/JuliaSmoothOptimizers/NLPModels.jl). See this [tutorial](https://jso.dev/NLPModelsJuMP.jl/dev/tutorial) for more details.
