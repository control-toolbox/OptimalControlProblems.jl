"""
$(TYPEDSIGNATURES)

Constructs an **OptimalControl problem** representing the Beam problem using the OptimalControl backend.  
The function sets up the state and control variables, boundary conditions, dynamics, path constraints, and the objective functional.  
It then performs direct transcription to generate a discrete optimal control problem (DOCP) and the corresponding nonlinear programming (NLP) model.

# Arguments

- `::OptimalControlBackend`: Placeholder type to specify the OptimalControl backend or solver interface.
- `N::Int=500`: (Keyword) Number of discretisation points for the direct transcription grid.

# Returns

- `docp`: The direct optimal control problem object, representing the discretised problem.
- `nlp`: The corresponding nonlinear programming model generated from the DOCP, ready for solver input.

# Example

```julia-repl
julia> using OptimalControlProblems

julia> docp, nlp = OptimalControlProblems.beam(OptimalControlBackend(); N=100);

julia> docp
DOCP object with 100 discretisation points

julia> nlp
NLP model corresponding to the beam problem
```

# References

- BOCOP repository: https://github.com/control-toolbox/bocop/tree/main/bocop
"""
function OptimalControlProblems.beam(::OptimalControlBackend; N::Int=500)

    # model
    ocp = @def begin
        t ∈ [0, 1], time
        x ∈ R², state
        u ∈ R, control
        x(0) == [0, 1]
        x(1) == [0, -1]
        ẋ(t) == [x₂(t), u(t)]
        0 ≤ x₁(t) ≤ 0.1
        -10 ≤ u(t) ≤ 5
        ∫(u(t)^2) → min
    end

    # initial guess
    init = (state=[0.05, 0.1], control=0.1)

    # DOCP and NLP
    docp = direct_transcription(ocp; init=init, grid_size=N, disc_method=:trapeze)
    nlp = model(docp)
    return docp, nlp
end
