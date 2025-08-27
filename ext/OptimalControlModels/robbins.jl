"""
$(TYPEDSIGNATURES)

Constructs an **OptimalControl problem** for the Robbins benchmark model.  
This function defines the state and control variables, system dynamics, initial and final conditions, and the cost functional, which minimises a weighted sum of the state and control contributions.  
It returns both a discretised direct optimal control problem (DOCP) and the corresponding nonlinear programming (NLP) model.  
Reference: [Robbins Problem on BOCOP](https://github.com/control-toolbox/bocop/tree/main/bocop)

# Arguments

- `::OptimalControlBackend`: Placeholder type specifying the OptimalControl backend or solver interface.
- `N::Int=500`: (Keyword) Number of discretisation points for the direct transcription grid.

# Returns

- `docp`: The direct optimal control problem object representing the Robbins problem.
- `nlp`: The corresponding nonlinear programming model obtained from the DOCP, suitable for numerical optimisation.

# Example

```julia-repl
julia> using OptimalControlProblems

julia> docp, nlp = OptimalControlProblems.robbins(OptimalControlBackend(); N=500);

julia> docp
DOCP object with 500 discretisation points

julia> nlp
NLP model corresponding to the Robbins problem
```
"""
function OptimalControlProblems.robbins(::OptimalControlBackend; N::Int=steps_number_data(:robbins))

    # parameters
    tf = final_time_data(:robbins)
    α = 3
    β = 0
    γ = 0.5

    # model
    ocp = @def begin
        t ∈ [0, tf], time
        x ∈ R³, state
        u ∈ R, control

        0 ≤ x[1](t) ≤ Inf

        x(0) == [1, -2, 0]
        x(tf) == [0, 0, 0]

        ẋ(t) == [x[2](t), x[3](t), u(t)]

        ∫(α * x[1](t) + β * x[1](t)^2 + γ * u(t)^2) → min
    end

    # initial guess
    xinit = [0.1, 0.1, 0.1]  # [x1, x2, x3]
    uinit = [0.1]  # [u]
    init = (state=xinit, control=uinit)

    # DOCP and NLP
    docp = direct_transcription(ocp; init=init, grid_size=N, disc_method=:trapeze)
    nlp = model(docp)

    return docp, nlp
end
