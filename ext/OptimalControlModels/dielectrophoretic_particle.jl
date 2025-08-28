"""
$(TYPEDSIGNATURES)

Constructs an **OptimalControl problem** representing a dielectrophoretic particle system.  
The function defines state and control variables, boundary conditions, and system dynamics, aiming to minimise the travel time of the particle between two points.  
It performs direct transcription to produce a discretised optimal control problem (DOCP) and the corresponding nonlinear programming (NLP) model.

# Arguments

- `::OptimalControlBackend`: Placeholder type specifying the OptimalControl backend or solver interface.
- `N::Int=500`: (Keyword) Number of discretisation points for the direct transcription grid.

# Returns

- `docp`: The direct optimal control problem object representing the dielectrophoretic particle problem.
- `nlp`: The corresponding nonlinear programming model obtained from the DOCP, suitable for numerical optimisation.

# Example

```julia-repl
julia> using OptimalControlProblems

julia> docp = OptimalControlProblems.dielectrophoretic_particle(OptimalControlBackend(); N=100);
```

# References

- Chang, D. E., Petit, N., & Rouchon, P. (2006). Time-optimal control of a particle in a dielectrophoretic system. *IEEE Transactions on Automatic Control*, 51(7), 1100-1114. [CPR2006]
- Formulation inspired by OptimalControl approach to time-optimal trajectory problems.
"""
function OptimalControlProblems.dielectrophoretic_particle(
    ::OptimalControlBackend, description::Symbol...; N::Int=steps_number_data(:dielectrophoretic_particle), kwargs...
)

    # parameters
    x0 = 1
    xf = 2
    α = -0.75
    c = 1

    ocp = @def begin
        tf ∈ R, variable
        t ∈ [0, tf], time
        q = (x, y) ∈ R², state
        u ∈ R, control

        x(0) == x0, (x0_con)
        y(0) == 0, (y0_con)
        x(tf) == xf, (xf_con)

        tf ≥ 0, (tf_con)
        -1 ≤ u(t) ≤ 1, (u_con)

        q̇(t) == dynamics(y(t), u(t))

        tf → min
    end

    function dynamics(y, u)
        return [y * u + α * u^2, -c * y + u]
    end

    # initial guess
    init = (state=[1, 1], control=0.1, variable=5)

    # DOCP and NLP
    docp = direct_transcription(ocp, description...; init=init, grid_size=N, disc_method=:trapeze, kwargs...)

    return docp
end
