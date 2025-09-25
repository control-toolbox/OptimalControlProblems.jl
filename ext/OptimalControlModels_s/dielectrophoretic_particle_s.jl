"""
$(TYPEDSIGNATURES)

Constructs an **OptimalControl problem** representing a dielectrophoretic particle system.  
The function defines state and control variables, boundary conditions, and system dynamics, aiming to minimise the travel time of the particle between two points.  
It performs direct transcription to produce a discretised optimal control problem (DOCP) and the corresponding nonlinear programming (NLP) model.

# Arguments

- `::OptimalControlBackend`: Placeholder type specifying the OptimalControl backend or solver interface.
- `grid_size::Int=500`: (Keyword) Number of discretisation points for the direct transcription grid.

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
function OptimalControlProblems.dielectrophoretic_particle_s(
    ::OptimalControlBackend,
    description::Symbol...;
    grid_size::Int=grid_size_data(:dielectrophoretic_particle),
    parameters::Union{Nothing, NamedTuple}=nothing,
    kwargs...,
)

    # parameters
    params = parameters_data(:dielectrophoretic_particle, parameters)
    t0 = params[:t0]
    x_t0 = params[:x_t0]
    y_t0 = params[:y_t0]
    x_tf = params[:x_tf]
    α = params[:α]
    c = params[:c]
    u_l = params[:u_l]
    u_u = params[:u_u]
    tf_l = params[:tf_l]

    ocp = @def begin
        tf ∈ R, variable
        t ∈ [t0, tf], time
        q = (x, y) ∈ R², state
        u ∈ R, control

        x(t0) == x_t0, (x_t0)
        y(t0) == y_t0, (y_t0)
        x(tf) == x_tf, (x_tf)
        tf ≥ tf_l, (tf_c)
        u_l ≤ u(t) ≤ u_u, (u_c)

        ∂(x)(t) == y(t) * u(t) + α * u(t)^2
        ∂(y)(t) == -c * y(t) + u(t)

        tf → min
    end

    # initial guess
    init = (state=[1, 1], control=0.1, variable=5)

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
