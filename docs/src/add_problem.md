# [Add a problem](@id add-problem)

To add a new problem to OptimalControlProblems, you need to follow these steps:

**1.** Create a new file in the `ext/MetaData` directory with the name of your problem and containing the needed information about the problem in a dictionary. For instance, if your problem is called `new_problem`, create a file named `new_problem.jl`. The dictionary should follow the template:

```julia
new_problem_meta = Dict(
    :name => "new_problem", # Name of the problem
    :nh => nothing,         # Number of discretization points
    :nvar => nothing,       # Number of variables
    :ncon => nothing,       # Number of constraints
    :minimize => true       # Whether the problem is a minimization problem
)
```
    
The use of `nothing` in the dictionary is allowed and is used to indicate that the information is not available yet. Once you have the information, you can update the dictionary with the correct values.

**2.** Define the OptimalControl model of the problem in another file in the `ext/OptimalControlModels` directory.

```julia
"""
    Description of the new problem
"""
function OptimalControlProblems.new_problem(::OptimalControlBackend; nh::Int=default_value)
    @def ocp begin
        # Define the problem here
        # ...
    end

    # Initial guess for the problem
    init = () 

    # Obtain the NLPModel + DOCP
    docp, nlp = direct_transcription(ocp; init=init, grid_size=nh, disc_method=:trapeze)

    return docp, nlp
end
```

**3.** Define the JuMP model of the problem in a new file in the `ext/JuMPModels` directory.

```julia
"""
    Description of the new problem
"""
function OptimalControlProblems.new_problem(::JuMPBackend; nh::Int=default_value)
    model = JuMP.Model()

    # Define the problem here
    # ...

    return model
end
```