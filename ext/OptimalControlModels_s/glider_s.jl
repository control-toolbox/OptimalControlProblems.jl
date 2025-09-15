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
function OptimalControlProblems.glider_s(
    ::OptimalControlBackend,
    description::Symbol...;
    grid_size::Int=steps_number_data(:glider),
    parameters::Union{Nothing, NamedTuple}=nothing,
    kwargs...,
)

    # parameters
    params = parameters_data(:glider, parameters)
    t0 = params[:t0]
    x_i = params[:x_i]
    y_i = params[:y_i]
    y_f = params[:y_f]
    vx_i = params[:vx_i]
    vx_f = params[:vx_f]
    vy_i = params[:vy_i]
    vy_f = params[:vy_f]
    u_c = params[:u_c]
    r_i = params[:r_i]
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
        x(t0) == x_i, (x0_i)
        y(t0) == y_i, (y0_i)
        vx(t0) == vx_i, (vx0_i)
        vy(t0) == vy_i, (vy0_i)

        # final conditions
        tf ≥ tf_l
        y(tf) == y_f, (yf_f)
        vx(tf) == vx_f, (vxf_f)
        vy(tf) == vy_f, (vyf_f)

        # dynamics
        r = (x(t) / r_i - 2.5)^2
        UpD = u_c * (1 - r) * exp(-r)
        w = vy(t) - UpD
        v = √(vx(t)^2 + w^2)
        D = 0.5 * (c0 + c1 * (cL(t)^2)) * ρ * S * (v^2)
        L = 0.5 * cL(t) * ρ * S * (v^2)

        ∂(x)(t) == vx(t)
        ∂(y)(t) == vy(t)
        ∂(vx)(t) == -(L * w + D * vx(t)) / (m * v) 
        ∂(vy)(t) == (L * vx(t) - D * w) / (m * v) - g

        # objective
        -x(tf) → min
    end

    # initial guess
    tfinit = 1
    xinit = t -> [x_i + vx_i * t / tfinit, y_i + t / tfinit * (y_f - y_i), vx_i, vy_i]
    uinit = cL_max / 2
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
