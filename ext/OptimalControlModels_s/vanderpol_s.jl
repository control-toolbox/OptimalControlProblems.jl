"""
$(TYPEDSIGNATURES)

Constructs an **OptimalControl problem** for the Van der Pol oscillator with a control input.  
The objective is to minimise a quadratic cost composed of the state and control effort over a fixed time horizon.  
The problem formulation can be found [here](https://github.com/control-toolbox/bocop/tree/main/bocop).

# Arguments

- `::OptimalControlBackend`: Placeholder type specifying the OptimalControl backend or solver interface.
- `N::Int=500`: (Keyword) Number of discretisation points for the direct transcription grid.

# Returns

- `docp`: The direct optimal control problem object representing the Van der Pol problem.
- `nlp`: The corresponding nonlinear programming model obtained from the DOCP, suitable for numerical optimisation.

# Example

```julia-repl
julia> using OptimalControlProblems

julia> docp = OptimalControlProblems.vanderpol(OptimalControlBackend(); N=500);
```
"""
function OptimalControlProblems.vanderpol_s(
    ::OptimalControlBackend,
    description::Symbol...;
    grid_size::Int=grid_size_data(:vanderpol),
    parameters::Union{Nothing, NamedTuple}=nothing,
    kwargs...,
)

    # parameters
    params = parameters_data(:vanderpol, parameters)
    t0 = params[:t0]
    tf = params[:tf]
    ω = params[:ω]
    ε = params[:ε]
    x₁_t0 = params[:x₁_t0]
    x₂_t0 = params[:x₂_t0]

    # model
    ocp = @def begin
        t ∈ [t0, tf], time
        x ∈ R², state
        u ∈ R, control

        x(t0) == [x₁_t0, x₂_t0]

        ∂(x₁)(t) == x₂(t)
        ∂(x₂)(t) == ε * ω * (1 - x₁(t)^2) * x₂(t) - ω^2 * x₁(t) + u(t)

        0.5∫(x₁(t)^2 + x₂(t)^2 + u(t)^2) → min
    end

    # initial guess
    xinit = [0.1, 0.1]  # [x₁, x₂]
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
