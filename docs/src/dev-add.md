# [Add a problem](@id add-problem)

To add a new problem to **OptimalControlProblems**, you must follow these steps:

**1.** Create a new file in the `ext/MetaData` directory with the name of your problem, containing the required information about the problem in a dictionary. For example, if your problem is called `new_problem`, create a file named `new_problem.jl`. The dictionary should follow the template:

```julia
new_problem_meta = OrderedDict(
    :grid_size => 100,                 # Number of steps
    :parameters => (
        t0 = 0,                        # Value of the initial time
        tf = 1,                        # Value of the final time
    ),
    # initial guess of every modeling, from the parameters: each component a constant,
    # a vector, or a function of the time t
    :init => p -> (state=t -> [p.t0 + t, 0.1], control=0.1),
)
```

The initial guess must make the solvers converge **without** being too close to the solution, so that the solves still exercise the iterations.

!!! note

    For more details about the metadata, see the [MetaData](@ref problems-introduction-metadata) section.

**2.** Define the **OptimalControl** model of the problem in a file named `new_problem.jl` in the `ext/OptimalControlModels` directory, following the template:

!!! tip

    Get inspiration from the already existing problems in [OptimalControlProblems.jl/ext/OptimalControlModels](https://github.com/control-toolbox/OptimalControlProblems.jl/tree/main/ext/OptimalControlModels). Especially if the final time is free.

```julia
"""
    Documentation of the method
"""
function OptimalControlProblems.new_problem(
    ::OptimalControlBackend;
    parameters::Union{Nothing, NamedTuple}=nothing,
    init=nothing,
)

    # parameters
    params = parameters_data(:new_problem, parameters)
    t0 = params[:t0]
    tf = params[:tf]

    # model
    @def ocp begin
        t ∈ [t0, tf], time
        ...
    end

    # initial guess: from the metadata, unless given
    init = OptimalControlProblems._initial_guess(:new_problem, params, init)

    # the continuous problem: it is discretised when solved
    return OptimalControlProblems._problem(:new_problem, :OptimalControl, ocp, init, params)

end
```

**3.** Define the scalarised **OptimalControl** model of the problem in a file named `new_problem.jl` in the `ext/OptimalControlScalarizedModels` directory. This version is similar to the previous case, ending with `OptimalControlProblems._problem(:new_problem, :OptimalControlScalarized, ocp, init, params)`, but, as an example, replace the dynamics:

```julia
ẋ(t) == [x₂(t), u(t)]
```

by

```julia
∂(x₁)(t) == x₂(t)
∂(x₂)(t) == u(t)
```

!!! warning

    The dynamics and the nonlinear constraints must be in scalar form. See the section [Dynamics (coordinatewise)](@extref OptimalControl modelling-abstract-syntax-dynamics-coord) and the following section for more details.

**4.** Define the **JuMP** model of the problem in a file named `new_problem.jl` in the `ext/JuMPModels` directory. The file declares the data of the problem only; the discretization is written once, in `ext/JuMPModels/schemes/`. Follow the template:

```julia
"""
    Documentation of the method
"""
function OptimalControlProblems.new_problem(
    ::JuMPBackend, args...;
    grid_size::Int=grid_size_data(:new_problem),
    scheme::Symbol=:trapeze,
    ordering::Symbol=:interleaved,
    parameters::Union{Nothing, NamedTuple}=nothing,
    init=nothing,
    kwargs...
)

    # parameters
    params = parameters_data(:new_problem, parameters)
    t0 = params[:t0]
    tf = params[:tf]

    data = ProblemData(;
        # names consistent with the other models, and bounds; the start values come from
        # the initial guess of the metadata
        state=[Component("x₁"), Component("x₂")],
        control=[Component("u"; lower=-1, upper=1)],
        costate=["∂x₁", "∂x₂"],
        times=v -> (t0, tf),
        dynamics=(x, u, v, t) -> [x[2], u[1]],
        lagrange=(x, u, v, t) -> u[1]^2,
        boundary=[(0, (x0, xf, v) -> x0[1], 0), (1, (x0, xf, v) -> xf[1], 1)],
    )
    init = OptimalControlProblems._initial_guess(:new_problem, params, init)
    return build(data, grid_size, scheme, ordering, args...; init=init, kwargs...)

end
```

!!! warning

    - The names of the components are the names of the JuMP containers (`model[:x₁]`) and must be consistent with the other models.
    - If `tf` is free, declare it as a variable and return it from `times`:

    ```julia
    variable=[Component("tf"; lower=0.1)],
    times=v -> (t0, v[1]),
    ```

    - Box constraints on the state, the control and the variable are bounds of the components; other constraints go in `path` (every node) and `boundary`, as `(lower, f, upper)`. The other field (`cost`) is described in the docstring of `ProblemData`.
    - The functions of `ProblemData` are written once for every scheme (`scheme`, see the page *JuMP transcription*): never assume that the dynamics are evaluated at the nodes only.

!!! tip

    Get inspiration from the already existing problems in [OptimalControlProblems.jl/ext/JuMPModels](https://github.com/control-toolbox/OptimalControlProblems.jl/tree/main/ext/JuMPModels). Especially if the final time is free.

**5.** Describe the problem in a file named `new_problem.md` in the `ext/Descriptions` directory. Get inspiration from the already existing descriptions in [OptimalControlProblems.jl/ext/Descriptions](https://github.com/control-toolbox/OptimalControlProblems.jl/tree/main/ext/Descriptions).
