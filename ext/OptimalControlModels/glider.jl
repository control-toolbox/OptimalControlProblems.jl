"""
$(TYPEDSIGNATURES)

Constructs an **OptimalControl problem** for a hang glider trajectory.  
The function defines state and control variables, glider dynamics in a thermal updraft, boundary conditions, and a cost functional aiming to maximise the final horizontal position.  
It returns both a discretised direct optimal control problem (DOCP) and the corresponding nonlinear programming (NLP) model.

# Arguments

- `::OptimalControlBackend`: Placeholder type specifying the OptimalControl backend or solver interface.
- `N::Int=500`: (Keyword) Number of discretisation points for the direct transcription grid.

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
    N::Int=steps_number_data(:glider),
    parameters::Union{Nothing, NamedTuple}=nothing,
    kwargs...,
)

    # parameters
    params = parameters_data(:glider, parameters)
    t0 = params[:t0]
    x_0 = params[:x_0]
    y_0 = params[:y_0]
    y_f = params[:y_f]
    vx_0 = params[:vx_0]
    vx_f = params[:vx_f]
    vy_0 = params[:vy_0]
    vy_f = params[:vy_f]
    u_c = params[:u_c]
    r_0 = params[:r_0]
    m = params[:m]
    g = params[:g]
    c0 = params[:c0]
    c1 = params[:c1]
    S = params[:S]
    ρ = params[:ρ]
    cL_min = params[:cL_min]
    cL_max = params[:cL_max]

    # model
    ocp = @def begin
        tf ∈ R, variable
        t ∈ [t0, tf], time
        z = (x, y, vx, vy) ∈ R⁴, state
        cL ∈ R, control

        # state constraints
        x(t) ≥ 0, (x_c)
        vx(t) ≥ 0, (vx_c)

        # control constraints
        cL_min ≤ cL(t) ≤ cL_max, (cL_c)

        # initial conditions
        x(t0) == x_0, (x0_i)
        y(t0) == y_0, (y0_i)
        vx(t0) == vx_0, (vx0_i)
        vy(t0) == vy_0, (vy0_i)

        # final conditions
        tf ≥ 0
        y(tf) == y_f, (yf_f)
        vx(tf) == vx_f, (vxf_f)
        vy(tf) == vy_f, (vyf_f)

        # dynamics
        ż(t) == dynamics(x(t), vx(t), vy(t), cL(t))

        # objective
        -x(tf) → min
    end

    function dynamics(x, vx, vy, cL)
        r = (x / r_0 - 2.5)^2
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
    tfinit = 1
    xinit = t -> [x_0 + vx_0 * t / tfinit, y_0 + t / tfinit * (y_f - y_0), vx_0, vy_0]
    uinit = cL_max / 2
    init = (state=xinit, control=uinit, variable=tfinit)

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
