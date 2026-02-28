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
function OptimalControlProblems.brachistochrone_s(
    ::OptimalControlBackend,
    description::Symbol...;
    grid_size::Int=grid_size_data(:brachistochrone),
    parameters::Union{Nothing,NamedTuple}=nothing,
    kwargs...,
)

    params = parameters_data(:brachistochrone, parameters)
    g  = params[:g]
    t0 = params[:t0]
    x0 = params[:x0]
    y0 = params[:y0]
    v0 = params[:v0]
    xf = params[:xf]
    yf = params[:yf]
    u_min = params[:u_min]
    u_max = params[:u_max]

    # model
    ocp = @def begin
        
        tf ∈ R, variable
        t ∈ [t0, tf], time

    
        z = (px, py, v) ∈ R³, state
        u ∈ [u_min, u_max], control

       
        z(t0) == [x0, y0, v0]
        
       
        px(tf) == xf
        py(tf) == yf

        0.1 ≤ tf ≤ 20.0

        ∂(px)(t) == v(t) * sin(u(t))
        ∂(py)(t) == -v(t) * cos(u(t))
        ∂(v)(t)  == g * cos(u(t))

        tf → min
    end

    # initial guess: linear interpolation to match JuMP
    tf_guess = 2.0
    init = (
        state = t -> [
            x0 + (t - t0) / (tf_guess - t0) * (xf - x0),
            y0 + (t - t0) / (tf_guess - t0) * (yf - y0),
            v0 + (t - t0) / (tf_guess - t0) * 10.0
        ],
        control = 1.57, 
        variable = tf_guess
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