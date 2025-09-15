"""
$(TYPEDSIGNATURES)

Constructs an **OptimalControl problem** for the Jackson benchmark model.  
The function sets up state and control variables, system dynamics, bounds, initial and final conditions, and a cost functional aimed at minimising the third state variable at final time.  
It returns both a discretised direct optimal control problem (DOCP) and the corresponding nonlinear programming (NLP) model.

# Arguments

- `::OptimalControlBackend`: Placeholder type specifying the OptimalControl backend or solver interface.
- `grid_size::Int=500`: (Keyword) Number of discretisation points for the direct transcription grid.

# Returns

- `docp`: The direct optimal control problem object representing the Jackson problem.
- `nlp`: The corresponding nonlinear programming model obtained from the DOCP, suitable for numerical optimisation.

# Example

```julia-repl
julia> using OptimalControlProblems

julia> docp = OptimalControlProblems.jackson(OptimalControlBackend(); N=500);
```

# References

- Problem formulation available at [Bocop repository](https://github.com/control-toolbox/bocop/tree/main/bocop)
"""
function OptimalControlProblems.jackson(
    ::OptimalControlBackend,
    description::Symbol...;
    grid_size::Int=grid_size_data(:jackson),
    parameters::Union{Nothing, NamedTuple}=nothing,
    kwargs...,
)

    # parameters
    params = parameters_data(:jackson, parameters)
    t0 = params[:t0]
    tf = params[:tf]
    k1 = params[:k1]
    k2 = params[:k2]
    k3 = params[:k3]
    a_l = params[:a_l]
    a_u = params[:a_u]
    b_l = params[:b_l]
    b_u = params[:b_u]
    x₃_l = params[:x₃_l]
    x₃_u = params[:x₃_u]
    u_l = params[:u_l]
    u_u = params[:u_u]
    a_t0 = params[:a_t0]
    b_t0 = params[:b_t0]
    x₃_t0 = params[:x₃_t0]

    # model
    ocp = @def begin
        t ∈ [t0, tf], time
        x = (a, b, x₃) ∈ R³, state
        u ∈ R, control

        x(t0) == [a_t0, b_t0, x₃_t0]

        [a_l, b_l, x₃_l] ≤ x(t) ≤ [a_u, b_u, x₃_u]
        u_l ≤ u(t) ≤ u_u

        ẋ(t) == [
            -u(t) * (k1 * a(t) - k2 * b(t)),
            u(t) * (k1 * a(t) - k2 * b(t)) - (1 - u(t)) * k3 * b(t),
            (1 - u(t)) * k3 * b(t),
        ]

        x[3](tf) → max
    end

    # initial guess
    xinit = [0.1, 0.1, 0.1]  # [a, b, x₃]
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
