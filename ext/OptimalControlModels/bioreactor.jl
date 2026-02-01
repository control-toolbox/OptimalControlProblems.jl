"""
$(TYPEDSIGNATURES)

Constructs an **OptimalControl problem** representing the Bioreactor problem.  
The function defines the state and control variables, boundary conditions, path constraints, growth and light dynamics, and an objective functional.  
It performs direct transcription to create a discretised optimal control problem (DOCP) and the corresponding nonlinear programming (NLP) model.

# Arguments

- `::OptimalControlBackend`: Placeholder type specifying the OptimalControl backend or solver interface.
- `grid_size::Int=500`: (Keyword) Number of discretisation points for the direct transcription grid.

# Returns

- `docp`: The direct optimal control problem object representing the discretised Bioreactor problem.
- `nlp`: The corresponding nonlinear programming model generated from the DOCP, ready for solver input.

# Example

```julia-repl
julia> using OptimalControlProblems

julia> docp = OptimalControlProblems.bioreactor(OptimalControlBackend(); N=100);
```

# References

- BOCOP repository: https://github.com/control-toolbox/bocop/tree/main/bocop
"""
function OptimalControlProblems.bioreactor(
    ::OptimalControlBackend,
    description::Symbol...;
    grid_size::Int=grid_size_data(:bioreactor),
    parameters::Union{Nothing,NamedTuple}=nothing,
    kwargs...,
)

    # parameters
    params = parameters_data(:bioreactor, parameters)
    t0 = params[:t0]
    tf = params[:tf]
    β = params[:β]
    c = params[:c]
    γ = params[:γ]
    halfperiod = params[:halfperiod]
    Ks = params[:Ks]
    μ2m = params[:μ2m]
    μbar = params[:μbar]
    r = params[:r]
    y_l = params[:y_l]
    s_l = params[:s_l]
    b_l = params[:b_l]
    u_l = params[:u_l]
    u_u = params[:u_u]
    y_t0_l = params[:y_t0_l]
    y_t0_u = params[:y_t0_u]
    s_t0_l = params[:s_t0_l]
    s_t0_u = params[:s_t0_u]
    b_t0_l = params[:b_t0_l]
    b_t0_u = params[:b_t0_u]


    # METHANE PROBLEM
    # μ2 according to growth model
    # μ according to light model
    # time scale is [0,10] for 24h (day then night)

    # growth model MONOD
    function growth(s, μ2m, Ks)
        return μ2m * s / (s + Ks)
    end

    # light model: max^2 (0,sin) * μbar
    # DAY/NIGHT CYCLE: [0,2 halfperiod] rescaled to [0,2pi]
    function light(time, halfperiod)
        days = time / (halfperiod * 2)
        tau = (days - floor(days)) * 2π
        return max(0, sin(tau))^2
    end

    
    # Model
    ocp = @def begin
        t ∈ [t0, tf], time
        x = (y, s, b) ∈ R³, state
        u ∈ R, control

        x(t) ≥ [0, 0, 1e-3]
        u_l ≤ u(t) ≤ u_u
        [y_t0_l, s_t0_l, b_t0_l] ≤ x(t0) ≤ [y_t0_u, s_t0_u, b_t0_u]

        μ = light(t, halfperiod) * μbar
        μ2 = growth(s(t), μ2m, Ks)

        ẋ[1](t) == μ * y(t) / (1 + y(t)) - (r + u(t)) * y(t)
        ẋ[2](t) == -μ2 * b(t) + u(t) * β * (γ * y(t) - s(t))
        ẋ[3](t) == (μ2 - u(t) * β) * b(t)

        -∫(μ2 * b(t) / (β + c)) → min
    end


    # initial guess
    init = (state=[0.15, 2.75, 1.75], control=0.5)

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
