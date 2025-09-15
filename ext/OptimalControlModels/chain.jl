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
function OptimalControlProblems.chain(
    ::OptimalControlBackend,
    description::Symbol...;
    grid_size::Int=grid_size_data(:chain),
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
    x₁_t0 = a
    x₂_t0 = params[:x₂_t0]
    x₃_t0 = params[:x₃_t0]
    x₁_tf = b
    x₃_tf = L

    # model
    ocp = @def begin
        t ∈ [t0, tf], time
        x ∈ R³, state
        u ∈ R, control

        # initial conditions
        x₁(t0) == x₁_t0, (x₁_t0)
        x₂(t0) == x₂_t0, (x₂_t0)
        x₃(t0) == x₃_t0, (x₃_t0)

        # final conditions
        x₁(tf) == x₁_tf, (x₁_tf)
        x₃(tf) == x₃_tf, (x₃_tf)

        # dynamics
        ẋ(t) == dynamics(x(t), u(t))

        # objective
        x₂(tf) → min
    end

    # dynamics
    function dynamics(x, u)
        return [u, x[1] * √(1 + u^2), √(1 + u^2)]
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
    uinit = t -> 4 * abs(b - a) * ((t - t0) / (tf - t0) - tmin)
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
