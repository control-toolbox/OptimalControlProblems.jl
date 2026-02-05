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
)
```

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
    ::OptimalControlBackend,
    description::Symbol...;
    grid_size::Int=grid_size_data(:new_problem),
    parameters::Union{Nothing, NamedTuple}=nothing,
    kwargs...,
)

    # parameters
    params = parameters_data(:beam, parameters)
    t0 = params[:t0]
    tf = params[:tf]

    # model
    @def ocp begin
        t ∈ [t0, tf], time
        ...
    end

    # initial guess for the problem
    init = () 

    # discretise the optimal control problem
    docp = direct_transcription(
        ocp,
        description...;
        lagrange_to_mayer=false,
        init=init,
        grid_size=grid_size,
        disc_method=:trapeze,
        kwargs...,
    )

    return docp

end
```

**3.** Define the scalarised **OptimalControl** model of the problem in a file named `new_problem_s.jl` in the `ext/OptimalControlModels_s` directory. This version is similar to the previous case but, as an example, replace the dynamics:

```julia
ẋ(t) == [x₂(t), u(t)]
```

by

```julia
∂(x₁)(t) == x₂(t)
∂(x₂)(t) == u(t)
```

!!! warning

    The dynamics and the nonlinear constraints must be in scalar form. See the section [Dynamics (coordinatewise)](@extref OptimalControl manual-abstract-dynamics-coord) and the following section for more details.

**4.** Define the **JuMP** model of the problem in a file named `new_problem.jl` in the `ext/JuMPModels` directory, following the template:

```julia
"""
    Documentation of the method
"""
function OptimalControlProblems.new_problem(
    ::JuMPBackend, args...; 
    grid_size::Int=grid_size_data(:new_problem),
    parameters::Union{Nothing, NamedTuple}=nothing,
    kwargs...
)

    # parameters
    params = parameters_data(:beam, parameters)
    t0 = params[:t0]
    tf = params[:tf]

    # model
    model = JuMP.Model(args...; kwargs...)

    # metadata: required
    model[:time_grid] = () -> range(t0, tf, grid_size+1) # tf is a fixed
    model[:state_components] = ["x₁", "x₂"]
    model[:costate_components] = ["∂x₁", "∂x₂"]
    model[:control_components] = ["u"]
    model[:variable_components] = String[]               # no variable

    # N = grid_size
    @expression(model, N, grid_size)

    # variables and initial guess
    @variables(
        model,
        begin
            x₁[0:N],    (start = 0.5) # consistent with model[:state_components]
            x₂[0:N],    (start = 0.1) # consistent with model[:state_components]
            u[0:N],     (start = 0.1) # consistent with model[:control_components]
        end
    )
    
    # @constraints, @objective...

    return model

end
```

!!! warning

    - The metadata in JuMP are required and must be consistent with the other models. 
    - If `tf` is free, then define:

    ```julia
    model[:time_grid] = () -> range(t0, value(model[:tf]), grid_size+1) # tf is a free
    ```

!!! tip

    Get inspiration from the already existing problems in [OptimalControlProblems.jl/ext/JuMPModels](https://github.com/control-toolbox/OptimalControlProblems.jl/tree/main/ext/JuMPModels). Especially if the final time is free.

**5.** Describe the problem in a file named `new_problem.md` in the `ext/Descriptions` directory. Get inspiration from the already existing descriptions in [OptimalControlProblems.jl/ext/Descriptions](https://github.com/control-toolbox/OptimalControlProblems.jl/tree/main/ext/Descriptions).
