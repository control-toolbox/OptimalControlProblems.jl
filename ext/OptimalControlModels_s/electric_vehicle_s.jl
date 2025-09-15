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
    grid_size::Int=steps_number_data(:electric_vehicle),
    parameters::Union{Nothing, NamedTuple}=nothing,
    kwargs...,
)

    # parameters
    params = parameters_data(:electric_vehicle, parameters)
    t0 = params[:t0]
    tf = params[:tf]
    b1 = params[:b1]
    b2 = params[:b2]
    h0 = params[:h0]
    h1 = params[:h1]
    h2 = params[:h2]
    α0 = params[:α0]
    α1 = params[:α1]
    α2 = params[:α2]
    α3 = params[:α3]
    x_i = params[:x_i]
    v_i = params[:v_i]
    x_f = params[:x_f]
    v_f = params[:v_f]

    # model
    ocp = @def begin
        t ∈ [t0, tf], time
        y = (x, v) ∈ R², state
        u ∈ R, control

        x(t0) == x_i, (x_i)
        v(t0) == v_i, (v_i)
        x(tf) == x_f, (x_f)
        v(tf) == v_f, (v_f)

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
        grid_size=grid_size,
        disc_method=:trapeze,
        kwargs...,
    )

    return docp
end
