"""
$(TYPEDSIGNATURES)

Constructs an **OptimalControl problem** for the Robertson's problem.
This problem is a stiff chemical kinetics problem.

# Arguments

- `::OptimalControlBackend`: Placeholder type specifying the OptimalControl backend or solver interface.
- `grid_size::Int=500`: (Keyword) Number of discretisation points for the direct transcription grid.

# Returns

- `docp`: The direct optimal control problem object representing the Robertson's problem.

# Example

```julia-repl
julia> using OptimalControlProblems

julia> docp = OptimalControlProblems.robertson(OptimalControlBackend(); N=500);
```
"""
function OptimalControlProblems.robertson(
    ::OptimalControlBackend,
    description::Symbol...;
    grid_size::Int=grid_size_data(:robertson),
    parameters::Union{Nothing,NamedTuple}=nothing,
    kwargs...,
)

    # parameters
    params = parameters_data(:robertson, parameters)
    t0 = params[:t0]
    tf = params[:tf]
    k₁ = params[:k₁]
    k₂ = params[:k₂]
    k₃ = params[:k₃]
    x_t0 = params[:x_t0]
    y_t0 = params[:y_t0]
    z_t0 = params[:z_t0]

    # model
    ocp = @def begin
        t ∈ [t0, tf], time
        x ∈ R³, state
        u ∈ R, control

        x(t0) == [x_t0, y_t0, z_t0]
        -1 ≤ u(t) ≤ 1

        ẋ(t) == [-k₁ * x[1](t) + k₂ * x[2](t) * x[3](t),
                  k₁ * x[1](t) - k₂ * x[2](t) * x[3](t) - k₃ * x[2](t)^2,
                  k₃ * x[2](t)^2]

        ∫(u(t)^2) → min
    end

    # initial guess
    init = (state=[x_t0, y_t0, z_t0], control=0.0)

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
