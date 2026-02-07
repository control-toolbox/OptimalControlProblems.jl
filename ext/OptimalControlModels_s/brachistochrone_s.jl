"""
$(TYPEDSIGNATURES)

Constructs an **OptimalControl problem** representing the Brachistochrone problem using the OptimalControl backend.
The function sets up the state and control variables, boundary conditions, dynamics, and the objective functional.
It then performs direct transcription to generate a discrete optimal control problem (DOCP).

# Arguments

- `::OptimalControlBackend`: Placeholder type to specify the OptimalControl backend or solver interface.
- `grid_size::Int=grid_size_data(:brachistochrone)`: (Keyword) Number of discretisation points for the direct transcription grid.
- `parameters::Union{Nothing,NamedTuple}=nothing`: (Keyword) Custom parameters to override defaults.

# Returns

- `docp`: The direct optimal control problem object, representing the discretised problem.

# Example

```julia-repl
julia> using OptimalControlProblems

julia> docp = OptimalControlProblems.brachistochrone(OptimalControlBackend(); N=100);
```

# References

- Dymos Brachistochrone: https://openmdao.github.io/dymos/examples/brachistochrone/brachistochrone.html
"""
function OptimalControlProblems.brachistochrone(
    ::OptimalControlBackend,
    description::Symbol...;
    grid_size::Int=grid_size_data(Val(:brachistochrone)),
    parameters::Union{Nothing,NamedTuple}=nothing,
    kwargs...,
)

   
    params = parameters_data(Val(:brachistochrone), parameters)
    g  = params[:g]
    t0 = params[:t0]
    x0 = params[:x0]
    y0 = params[:y0]
    v0 = params[:v0]
    xf = params[:xf]
    yf = params[:yf]

    # model
    ocp = @def begin
        
        tf ∈ R, variable
        t ∈ [t0, tf], time

        x ∈ R, state
        y ∈ R, state
        v ∈ R, state

        u ∈ R, control

        x(t0) == x0
        y(t0) == y0
        v(t0) == v0
        
        x(tf) == xf
        y(tf) == yf

        0.1 ≤ tf ≤ 20.0

        ∂(x)(t) == v(t) * sin(u(t))
        ∂(y)(t) == v(t) * cos(u(t))
        ∂(v)(t) == g * cos(u(t))

        tf → min
    end

    # initial guess
    init = (
        state = [5.0, 7.5, 5.0], 
        control = 1.57, 
        variable = 2.0
    )

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