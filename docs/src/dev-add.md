# [Add a problem](@id add-problem)

To add a new problem to **OptimalControlProblems**, you should follow these steps:

**1.** Create a new file in the `ext/MetaData` directory with the name of your problem, containing the required information about the problem in a dictionary. For example, if your problem is called `new_problem`, create a file named `new_problem.jl`. The dictionary should follow the template:

```julia
new_problem_meta = OrderedDict(
    :name => "new_problem",             # Problem name
    :N => 100,                          # Number of steps
    :minimise => true,                  # Whether the objective is minimised (true) or maximised (false)
    :state_name => ["x1", "x2"],        # Names of the state components
    :costate_name => ["∂x1", "∂x2"],    # Names of the costate components (dual variables of the dynamics constraints)
    :control_name => ["u"],             # Names of the control components
    :variable_name => ["v"],            # Names of the optimisation variables
    :final_time => (:fixed, 1),         # Final time information
)
```

!!! note

    For more details about the metadata, see the [MetaData](@ref problems-introduction-metadata) section.

**2.** Define the DOCP **OptimalControl** model of the problem in a separate file in the `ext/OptimalControlModels` directory.

```julia
"""
    Description of the new problem
"""
function OptimalControlProblems.new_problem(::OptimalControlBackend, description::Symbol...; N::Int=steps_number_data(:new_problem), kwargs...)

    # if tf is fixed
    tf = final_time_data(:new_problem)

    # model
    @def ocp begin
        # Define the problem here
        # ...
    end

    # initial guess for the problem
    init = () 

    # DOCP and NLP
    docp = direct_transcription(ocp, description...; init=init, grid_size=N, disc_method=:trapeze, kwargs...)

    return docp

end
```

**3.** Define the NLP **JuMP** model of the problem in a new file in the `ext/JuMPModels` directory.

```julia
"""
    Description of the new problem
"""
function OptimalControlProblems.new_problem(::JuMPBackend, args...; N::Int=steps_number_data(:new_problem), kwargs...)

    # if tf is fixed
    tf = final_time_data(:new_problem)

    # model
    model = JuMP.Model(args...; kwargs...)

    # Define the problem here
    # ...

    return model
end
```
