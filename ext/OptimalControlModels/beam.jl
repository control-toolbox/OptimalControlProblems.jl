"""
$(TYPEDSIGNATURES)

Constructs an OptimalControl problem representing the Beam problem using the OptimalControl backend, as formulated in the BOCOP [repository](https://github.com/control-toolbox/bocop/tree/main/bocop).
The model includes state and control variables, boundary conditions, dynamics, path constraints, 
and an objective functional.

# Arguments

- `::OptimalControlBackend`: Placeholder for the OptimalControl backend type, selecting the solver interface.
- `N::Int=500`: Number of discretisation points for the direct transcription.

# Returns

- `docp`: The direct optimal control problem (DOCP) object.
- `nlp`: The corresponding nonlinear programming (NLP) model obtained from direct transcription.

# Example

```julia-repl
julia> docp, nlp = OptimalControlProblems.beam(OptimalControlBackend(); N=500)
(DOCP object, NLP model)
```
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
