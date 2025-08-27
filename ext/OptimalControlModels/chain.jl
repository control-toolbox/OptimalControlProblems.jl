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

julia> docp, nlp = OptimalControlProblems.chain(OptimalControlBackend(); N=100);

julia> docp
DOCP object with 100 discretisation points

julia> nlp
NLP model corresponding to the Hanging Chain problem
```

# References

- Formulation inspired by OptimalControl approach to variational problems and chain equilibrium.
- Original problem source: [BOCOP repository](https://github.com/control-toolbox/bocop/tree/main/bocop)
"""
function OptimalControlProblems.chain(::OptimalControlBackend; N::Int=steps_number_data(:chain))

    # parameters
    L = 4
    a = 1
    b = 3
    tf = 1

    # model
    ocp = @def begin

        #
        t ∈ [0, tf], time
        x ∈ R³, state
        u ∈ R, control

        # initial conditions
        x₁(0) == a, (x1_ic)
        x₂(0) == 0, (x2_ic)
        x₃(0) == 0, (x3_ic)

        # final conditions
        x₁(tf) == b, (x1_con)
        x₃(tf) == L, (x3_con)

        # dynamics
        ẋ(t) == dynamics(x(t), u(t))

        # objective
        x₂(tf) → min
    end

    # dynamics
    function dynamics(x, u)
        return [u, x[1] * √(1 + u^2), √(1 + u^2)]
    end

    # Initial guess
    tmin = b > a ? 1 / 4 : 3 / 4
    xinit =
        t -> [
            4 * abs(b - a) * t / tf * (0.5 * t / tf - tmin) + a,
            (4 * abs(b - a) * t / tf * (0.5 * t / tf - tmin) + a) *
            (4 * abs(b - a) * (t / tf - tmin)),
            4 * abs(b - a) * (t / tf - tmin),
        ]
    uinit = t -> 4 * abs(b - a) * (t / tf - tmin)
    init = (state=xinit, control=uinit)

    # NLPModel + DOCP
    docp = direct_transcription(ocp; init=init, grid_size=N, disc_method=:trapeze)
    nlp = model(docp)
    return docp, nlp
end
