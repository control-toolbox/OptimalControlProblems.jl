# [Add a problem](@id add-problem)

To add a new problem to OptimalControlProblems, you need to follow these steps:

**1.** Create a new file in the `ext/MetaData` directory with the name of your problem and containing the needed information about the problem in a dictionary. For instance, if your problem is called `new_problem`, create a file named `new_problem.jl`. The dictionary should follow the template:

```julia
new_problem_meta = OrderedDict(
    :name => "new_problem",             # Name of the problem
    :N => 100,                          # Number of discretization points
    :minimize => true                   # Whether the problem is a min problem or not
    :state_name => ["x1", "x2"],        # Names of the components of the state
    :costate_name => ["∂x1", "∂x2"],    # Names of the dynamics constraints
    :control_name => ["u"],             # Names of the components of the control
    :time => ("final_time", "tf", 1),   # Value of the final time.
)
```
    
!!! note

    For more details about the metadata, we refer to the [MetaData](@ref problems-introduction-metadata) section.

**2.** Define the OptimalControl model of the problem in another file in the `ext/OptimalControlModels` directory.

```julia
"""
    Description of the new problem
"""
function OptimalControlProblems.new_problem(::OptimalControlBackend; N::Int=default_value)

    # model
    @def ocp begin
        # Define the problem here
        # ...
    end

    # initial guess for the problem
    init = () 

    # DOCP and NLP
    docp, nlp = direct_transcription(ocp; init=init, grid_size=N, disc_method=:trapeze)

    return docp, nlp

end
```

**3.** Define the JuMP model of the problem in a new file in the `ext/JuMPModels` directory.

```julia
"""
    Description of the new problem
"""
function OptimalControlProblems.new_problem(::JuMPBackend; N::Int=default_value)

    # model
    model = JuMP.Model()

    # Define the problem here
    # ...

    return model
end
```