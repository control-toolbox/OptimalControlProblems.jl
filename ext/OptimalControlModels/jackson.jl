"""
$(TYPEDSIGNATURES)

Constructs an **OptimalControl problem** for the Jackson benchmark model.  
The function sets up state and control variables, system dynamics, bounds, initial and final conditions, and a cost functional aimed at minimising the third state variable at final time.  
It returns both a discretised direct optimal control problem (DOCP) and the corresponding nonlinear programming (NLP) model.

# Arguments

- `::OptimalControlBackend`: Placeholder type specifying the OptimalControl backend or solver interface.
- `N::Int=500`: (Keyword) Number of discretisation points for the direct transcription grid.

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
    ::OptimalControlBackend; N::Int=steps_number_data(:jackson)
)

    # parameters
    tf = final_time_data(:jackson)
    k1 = 1
    k2 = 10
    k3 = 1

    # model
    ocp = @def begin
        t ∈ [0, tf], time
        x ∈ R³, state
        u ∈ R, control

        a = x[1]
        b = x[2]

        x(0) == [1, 0, 0]

        [0, 0, 0] ≤ x(t) ≤ [1.1, 1.1, 1.1]
        0 ≤ u(t) ≤ 1

        ẋ(t) == [
            -u(t) * (k1 * a(t) - k2 * b(t)),
            u(t) * (k1 * a(t) - k2 * b(t)) - (1 - u(t)) * k3 * b(t),
            (1 - u(t)) * k3 * b(t),
        ]

        -x[3](tf) → min
    end

    # initial guess
    xinit = [0.1, 0.1, 0.1]  # [a, b, x3]
    uinit = [0.1]  # [u]
    init = (state=xinit, control=uinit)

    # DOCP and NLP
    docp = direct_transcription(ocp; init=init, grid_size=N, disc_method=:trapeze)

    return docp
end
