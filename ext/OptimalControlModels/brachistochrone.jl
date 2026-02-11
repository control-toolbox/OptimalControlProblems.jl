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
    
    params = parameters_data(:brachistochrone, parameters)
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

        z = (px, py, v) ∈ R³, state
        u ∈ R, control

        z(t0) == [x0, y0, v0]
        
        px(tf) == xf
        py(tf) == yf

        0.1 ≤ tf ≤ 20.0 

        ṗx(t) == v(t) * sin(u(t))
        ṗy(t) == v(t) * cos(u(t))
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