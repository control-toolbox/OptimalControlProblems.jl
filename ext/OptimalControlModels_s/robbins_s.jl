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

julia> docp = OptimalControlProblems.robbins(OptimalControlBackend(); N=500);
```
"""
function OptimalControlProblems.robbins_s(
    ::OptimalControlBackend,
    description::Symbol...;
    N::Int=steps_number_data(:robbins),
    parameters::Union{Nothing, NamedTuple}=nothing,
    kwargs...,
)

    # parameters
    params = parameters_data(:robbins, parameters)
    t0 = params[:t0]
    tf = params[:tf]
    α = params[:α]
    β = params[:β]
    γ = params[:γ]

    # model
    ocp = @def begin
        t ∈ [t0, tf], time
        x ∈ R³, state
        u ∈ R, control

        0 ≤ x[1](t) ≤ Inf

        x(t0) == [1, -2, 0]
        x(tf) == [0, 0, 0]

        ∂(x₁)(t) == x₂(t)
        ∂(x₂)(t) == x₃(t)
        ∂(x₃)(t) == u(t)

        ∫(α * x₁(t) + β * x₁(t)^2 + γ * u(t)^2) → min
    end

    # initial guess
    xinit = [0.1, 0.1, 0.1]  # [x1, x2, x3]
    uinit = [0.1]  # [u]
    init = (state=xinit, control=uinit)

    # discretise the optimal control problem
    docp = direct_transcription(
        ocp,
        description...;
        lagrange_to_mayer=false,
        init=init,
        grid_size=N,
        disc_method=:trapeze,
        kwargs...,
    )

    return docp
end
