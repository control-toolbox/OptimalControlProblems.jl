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
    ::OptimalControlBackend, description::Symbol...; N::Int=steps_number_data(:glider), kwargs...
)

    # parameters
    x_0 = 0
    y_0 = 1000
    y_f = 900
    vx_0 = 13.23
    vx_f = 13.23
    vy_0 = -1.288
    vy_f = -1.288
    u_c = 2.5
    r_0 = 100
    m = 100
    g = 9.81
    c0 = 0.034
    c1 = 0.069662
    S = 14
    ρ = 1.13
    cL_min = 0
    cL_max = 1.4

    # model
    ocp = @def begin
        tf ∈ R, variable
        t ∈ [0, tf], time
        z = (x, y, vx, vy) ∈ R⁴, state
        cL ∈ R, control

        # state constraints
        x(t) ≥ 0, (x_con)
        vx(t) ≥ 0, (vx_con)

        # control constraints
        cL_min ≤ cL(t) ≤ cL_max, (cL_con)

        # initial conditions
        x(0) == x_0, (x0_con)
        y(0) == y_0, (y0_con)
        vx(0) == vx_0, (vx0_con)
        vy(0) == vy_0, (vy0_con)

        # final conditions
        tf ≥ 0
        y(tf) == y_f, (yf_con)
        vx(tf) == vx_f, (vxf_con)
        vy(tf) == vy_f, (vyf_con)

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

    # Initial guess
    tfinit = 1
    xinit = t -> [x_0 + vx_0 * t / tfinit, y_0 + t / tfinit * (y_f - y_0), vx_0, vy_0]
    uinit = cL_max / 2
    init = (state=xinit, control=uinit, variable=tfinit)

    # DOCP and NLP
        docp = direct_transcription(
        ocp, 
        description...; 
        lagrange_to_mayer=false,
        init=init, 
        grid_size=N, 
        disc_method=:trapeze, 
        kwargs...
    )

    return docp
end
