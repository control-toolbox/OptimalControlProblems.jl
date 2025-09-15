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

julia> docp = OptimalControlProblems.beam(OptimalControlBackend(); N=100);
```

# References

- BOCOP repository: https://github.com/control-toolbox/bocop/tree/main/bocop
"""
function OptimalControlProblems.beam(
    ::OptimalControlBackend,
    description::Symbol...;
    grid_size::Int=steps_number_data(:beam),
    parameters::Union{Nothing, NamedTuple}=nothing,
    kwargs...,
)

    # parameters
    params = parameters_data(:beam, parameters)
    t0 = params[:t0]
    tf = params[:tf]
    x_t0 = params[:x_t0]
    x_tf = params[:x_tf]
    x₁_l = params[:x₁_l]
    x₁_u = params[:x₁_u]

    # model
    ocp = @def begin
        t ∈ [t0, tf], time
        x ∈ R², state
        u ∈ R, control
        x(t0) == x_t0
        x(tf) == x_tf
        ẋ(t) == [x₂(t), u(t)]
        x₁_l ≤ x₁(t) ≤ x₁_u
        ∫(u(t)^2) → min
    end

    # initial guess
    init = (state=[0.05, 0.1], control=0.1)

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
