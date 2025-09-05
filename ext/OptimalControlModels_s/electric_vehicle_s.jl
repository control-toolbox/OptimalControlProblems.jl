"""
$(TYPEDSIGNATURES)

Constructs an **OptimalControl problem** for an electric vehicle trajectory.  
The function defines state and control variables, vehicle dynamics, boundary conditions, and a cost functional representing energy and control effort.  
It returns both a discretised direct optimal control problem (DOCP) and the corresponding nonlinear programming (NLP) model.

# Arguments

- `::OptimalControlBackend`: Placeholder type specifying the OptimalControl backend or solver interface.
- `N::Int=500`: (Keyword) Number of discretisation points for the direct transcription grid.

# Returns

- `docp`: The direct optimal control problem object representing the electric vehicle trajectory optimisation.
- `nlp`: The corresponding nonlinear programming model obtained from the DOCP, suitable for numerical optimisation.

# Example

```julia-repl
julia> using OptimalControlProblems

julia> docp = OptimalControlProblems.electric_vehicle(OptimalControlBackend(); N=500);
```

# References

- Nicolas Petit and Antonio Sciarretta. "Optimal drive of electric vehicles using an inversion-based trajectory generation approach." IFAC Proceedings Volumes 44, no. 1 (2011): 14519-14526. [PS2011]
- Problem instance follows OptimalControl formulation for electric vehicle trajectory optimisation.
"""
function OptimalControlProblems.electric_vehicle_s(
    ::OptimalControlBackend,
    description::Symbol...;
    N::Int=steps_number_data(:electric_vehicle),
    kwargs...,
)

    # parameters
    tf = final_time_data(:electric_vehicle)
    D = 10
    b1 = 1e0
    b2 = 1e0
    h0 = 0.1
    h1 = 1
    h2 = 1e-3
    α0, α1, α2, α3 = (3, 0.4, -1, 0.1)

    # model
    ocp = @def begin
        t ∈ [0, tf], time
        y = (x, v) ∈ R², state
        u ∈ R, control

        x(0) == 0, (x_i)
        v(0) == 0, (v_i)
        x(tf) == D, (x_f)
        v(tf) == 0, (v_f)

        road = α0 + α1 * x(t) + α2 * x(t)^2 + α3 * x(t)^3
        ∂(x)(t) == v(t)
        ∂(v)(t) == h1 * u(t) - h2 * v(t)^2 - h0 - road

        ∫(b1 * u(t) * v(t) + b2 * u(t)^2) → min
    end


    # initial guess
    yinit = [0.1, 0.1]  # [x, v]
    uinit = [0.1]       # [u]
    init = (state=yinit, control=uinit)

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
