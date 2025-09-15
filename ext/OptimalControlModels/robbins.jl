"""
$(TYPEDSIGNATURES)

Constructs an **OptimalControl problem** for the Robbins benchmark model.  
This function defines the state and control variables, system dynamics, initial and final conditions, and the cost functional, which minimises a weighted sum of the state and control contributions.  
It returns both a discretised direct optimal control problem (DOCP) and the corresponding nonlinear programming (NLP) model.  
Reference: [Robbins Problem on BOCOP](https://github.com/control-toolbox/bocop/tree/main/bocop)

# Arguments

- `::OptimalControlBackend`: Placeholder type specifying the OptimalControl backend or solver interface.
- `grid_size::Int=500`: (Keyword) Number of discretisation points for the direct transcription grid.

# Returns

- `docp`: The direct optimal control problem object representing the Robbins problem.
- `nlp`: The corresponding nonlinear programming model obtained from the DOCP, suitable for numerical optimisation.

# Example

```julia-repl
julia> using OptimalControlProblems

julia> docp = OptimalControlProblems.robbins(OptimalControlBackend(); N=500);
```
"""
function OptimalControlProblems.robbins(
    ::OptimalControlBackend,
    description::Symbol...;
    grid_size::Int=grid_size_data(:robbins),
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
    x₁_l = params[:x₁_l]
    x₁_t0 = params[:x₁_t0]
    x₂_t0 = params[:x₂_t0]
    x₃_t0 = params[:x₃_t0]
    x₁_tf = params[:x₁_tf]
    x₂_tf = params[:x₂_tf]
    x₃_tf = params[:x₃_tf]

    # model
    ocp = @def begin
        t ∈ [t0, tf], time
        x ∈ R³, state
        u ∈ R, control

        x[1](t) ≥ x₁_l

        x(t0) == [x₁_t0, x₂_t0, x₃_t0]
        x(tf) == [x₁_tf, x₂_tf, x₃_tf]

        ẋ(t) == [x[2](t), x[3](t), u(t)]

        ∫(α * x[1](t) + β * x[1](t)^2 + γ * u(t)^2) → min
    end

    # initial guess
    xinit = [0.1, 0.1, 0.1]  # [x₁, x₂, x₃]
    uinit = [0.1]  # [u]
    init = (state=xinit, control=uinit)

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
