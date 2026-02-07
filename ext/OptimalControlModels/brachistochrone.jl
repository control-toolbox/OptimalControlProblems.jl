"""
$(TYPEDSIGNATURES)

Constructs the **Brachistochrone** optimal control problem.
The goal is to move from point A to point B in minimum time under gravity.

# Arguments
- `::OptimalControlBackend`: The backend type.
- `grid_size`: Number of time steps.
- `parameters`: Optional parameter overrides.

```julia-repl
julia> using OptimalControlProblems

julia> docp = OptimalControlProblems.brachistochrone(OptimalControlBackend(); N=100);
```
"""
function OptimalControlProblems.brachistochrone(
    ::OptimalControlBackend,
    description::Symbol...;
    grid_size::Int=grid_size_data(:brachistochrone),
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

        ẋ(t) == v(t) * sin(u(t))
        ẏ(t) == v(t) * cos(u(t))
        v̇(t) == g * cos(u(t))

        tf → min
    end

 
    init = (
        state = [5.0, 7.5, 5.0], 
        control = 1.5, 
        variable = 1.0
    )

    docp = direct_transcription(
        ocp,
        description...;
        init = init,
        grid_size = grid_size,
        kwargs...
    )

    return docp
end
