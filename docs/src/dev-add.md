# [Add a problem](@id add-problem)

To add a new problem to **OptimalControlProblems**, you must follow these steps:

**1.** Create a new file in the `ext/MetaData` directory with the name of your problem, containing the required information about the problem in a dictionary. For example, if your problem is called `new_problem`, create a file named `new_problem.jl`. The dictionary should follow the template:

```julia
new_problem_meta = OrderedDict(
    :grid_size => 100,                         # Number of steps                 # Whether we minimise (true) or maximise (false)
    :state_name => ["x1", "x2"],       # Names of the state components
    :costate_name => ["∂x1", "∂x2"],   # Names of the dynamics constraints (for the costate)
    :control_name => ["u"],            # Names of the control components
    :variable_name => ["v"],           # Names of the optimisation variables
    :parameters => (tf = 1, ),         # Final time information
)
```

!!! note

    For more details about the metadata, see the [MetaData](@ref problems-introduction-metadata) section.

**2.** Define the **OptimalControl** model of the problem in a file named `new_problem.jl` in the `ext/OptimalControlModels` directory, following the template:

```julia
"""
    Documentation of the method
"""
function OptimalControlProblems.new_problem(
    ::OptimalControlBackend,
    description::Symbol...;
    grid_size::Int=steps_number_data(:new_problem),
    kwargs...,
)

    # if tf is fixed
    tf = final_time_data(:new_problem)

    # model
    @def ocp begin
        # Define the problem here
        # ...
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
        kwargs...
    )

    return docp

end
```

**3.** Define the **JuMP** model of the problem in a file named `new_problem.jl` in the `ext/JuMPModels` directory, following the template:

```julia
"""
    Documentation of the method
"""
function OptimalControlProblems.new_problem(
    ::JuMPBackend, args...; grid_size::Int=steps_number_data(:new_problem), kwargs...
)

    # if tf is fixed
    tf = final_time_data(:new_problem)

    # model
    model = JuMP.Model(args...; kwargs...)

    # define the problem
    # @variables, @constraints, @objective...

    return model

end
```

**4.** Describe the problem in a file named `new_problem.jl` in the `ext/Descriptions` directory. Please get inspiration from the already existing descriptions in [OptimalControlProblems.jl/ext/Descriptions](https://github.com/control-toolbox/OptimalControlProblems.jl/tree/main/ext/Descriptions).
