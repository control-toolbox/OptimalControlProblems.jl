"""
$(TYPEDSIGNATURES)

Constructs an **OptimalControl problem** for a hang glider trajectory.  
The function defines state and control variables, glider dynamics in a thermal updraft, boundary conditions, and a cost functional aiming to maximise the final horizontal position.  
It returns both a discretised direct optimal control problem (DOCP) and the corresponding nonlinear programming (NLP) model.

# Arguments

- `::OptimalControlBackend`: Placeholder type specifying the OptimalControl backend or solver interface.
- `grid_size::Int=500`: (Keyword) Number of discretisation points for the direct transcription grid.

# Returns

- `docp`: The direct optimal control problem object representing the hang glider trajectory optimisation.
- `nlp`: The corresponding nonlinear programming model obtained from the DOCP, suitable for numerical optimisation.

# Example

```julia-repl
julia> using OptimalControlProblems

julia> docp = OptimalControlProblems.glider(OptimalControlBackend(); N=500);
```

# References

- Original formulation from MadNLP/COPSBenchmark.
- Problem inspired by glider dynamics with thermal updraft and lift modelling.
"""
function OptimalControlProblems.glider(
    ::OptimalControlBackend,
    description::Symbol...;
    grid_size::Int=grid_size_data(:glider),
    parameters::Union{Nothing,NamedTuple}=nothing,
    kwargs...,
)

    # parameters
    params = parameters_data(:glider, parameters)
    t0 = params[:t0]
    x_t0 = params[:x_t0]
    y_t0 = params[:y_t0]
    y_tf = params[:y_tf]
    vx_t0 = params[:vx_t0]
    vx_tf = params[:vx_tf]
    vy_t0 = params[:vy_t0]
    vy_tf = params[:vy_tf]
    u_c = params[:u_c]
    r_t0 = params[:r_t0]
    m = params[:m]
    g = params[:g]
    c0 = params[:c0]
    c1 = params[:c1]
    S = params[:S]
    ρ = params[:ρ]
    cL_min = params[:cL_min]
    cL_max = params[:cL_max]
    tf_l = params[:tf_l]
    x_l = params[:x_l]
    vx_l = params[:vx_l]

    # model
    ocp = @def begin
        tf ∈ R, variable
        t ∈ [t0, tf], time
        z = (x, y, vx, vy) ∈ R⁴, state
        cL ∈ R, control

        # state constraints
        x(t) ≥ x_l, (x_c)
        vx(t) ≥ vx_l, (vx_c)

        # control constraints
        cL_min ≤ cL(t) ≤ cL_max, (cL_c)

        # initial conditions
        x(t0) == x_t0, (x0_t0)
        y(t0) == y_t0, (y0_t0)
        vx(t0) == vx_t0, (vx0_t0)
        vy(t0) == vy_t0, (vy0_t0)

        # final conditions
        tf ≥ tf_l
        y(tf) == y_tf, (yf_tf)
        vx(tf) == vx_tf, (vxf_tf)
        vy(tf) == vy_tf, (vyf_tf)

        # dynamics
        ż(t) == dynamics(x(t), vx(t), vy(t), cL(t))

        # objective
        -x(tf) → min
    end

    function dynamics(x, vx, vy, cL)
        r = (x / r_t0 - 2.5)^2
        UpD = u_c * (1 - r) * exp(-r)
        w = vy - UpD
        v = √(vx^2 + w^2)
        D = 0.5 * (c0 + c1 * (cL^2)) * ρ * S * (v^2)
        L = 0.5 * cL * ρ * S * (v^2)

        ∂x = vx
        ∂y = vy
        ∂vx = -(L * w + D * vx) / (m * v)
        ∂vy = (L * vx - D * w) / (m * v) - g

        return [∂x, ∂y, ∂vx, ∂vy]
    end

    # initial guess
    tfinit = 100.0 
    xinit = t -> [
        x_t0 + (t / tfinit) * 1248.0,                         
        y_t0 + (t / tfinit) * (y_tf - y_t0),         
        vx_t0 + (t / tfinit) * (vx_tf - vx_t0),       
        vy_t0 + (t / tfinit) * (vy_tf - vy_t0)        
    ]
    uinit = 1.0
    init = (state=xinit, control=uinit, variable=tfinit) 

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
