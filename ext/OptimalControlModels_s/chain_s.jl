"""
$(TYPEDSIGNATURES)

Constructs an **OptimalControl problem** representing the Hanging Chain problem.  
The function defines state and control variables, boundary conditions, and system dynamics, with the objective of minimising the vertical displacement of the chain's midpoint.  
It performs direct transcription to produce a discretised optimal control problem (DOCP) and the corresponding nonlinear programming (NLP) model.

# Arguments

- `::OptimalControlBackend`: Placeholder type specifying the OptimalControl backend or solver interface.
- `N::Int=500`: (Keyword) Number of discretisation points for the direct transcription grid.

# Returns

- `docp`: The direct optimal control problem object representing the discretised Hanging Chain problem.
- `nlp`: The corresponding nonlinear programming model obtained from the DOCP, suitable for numerical optimisation.

# Example

```julia-repl
julia> using OptimalControlProblems

julia> docp = OptimalControlProblems.chain(OptimalControlBackend(); N=100);
```

# References

- Formulation inspired by OptimalControl approach to variational problems and chain equilibrium.
- Original problem source: [BOCOP repository](https://github.com/control-toolbox/bocop/tree/main/bocop)
"""
function OptimalControlProblems.chain_s(
    ::OptimalControlBackend,
    description::Symbol...;
    N::Int=steps_number_data(:chain),
    parameters::Union{Nothing, NamedTuple}=nothing,
    kwargs...,
)

    # parameters
    params = parameters_data(:chain, parameters)
    t0 = params[:t0]
    tf = params[:tf]
    L = params[:L]
    a = params[:a]
    b = params[:b]

    # model
    ocp = @def begin
        t ∈ [t0, tf], time
        x ∈ R³, state
        u ∈ R, control

        # initial conditions
        x₁(t0) == a, (x1_i)
        x₂(t0) == 0, (x2_i)
        x₃(t0) == 0, (x3_i)

        # final conditions
        x₁(tf) == b, (x1_f)
        x₃(tf) == L, (x3_f)

        # dynamics
        ∂(x₁)(t) == u(t)
        ∂(x₂)(t) == x₁(t) * √(1 + u(t)^2)
        ∂(x₃)(t) == √(1 + u(t)^2)

        # objective
        x₂(tf) → min
    end

    # initial guess
    tmin = b > a ? 1 / 4 : 3 / 4
    xinit =
        t -> [
            4 * abs(b - a) * (t - t0) / (tf - t0) * (0.5 * (t - t0) / (tf - t0) - tmin) + a,
            (4 * abs(b - a) * (t - t0) / (tf - t0) * (0.5 * (t - t0) / (tf - t0) - tmin) + a) *
            (4 * abs(b - a) * ((t - t0) / (tf - t0) - tmin)),
            4 * abs(b - a) * ((t - t0) / (tf - t0) - tmin),
        ]
    uinit = t -> 4 * abs(b - a) * (t / tf - tmin)
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
