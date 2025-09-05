"""
$(TYPEDSIGNATURES)

Constructs an **OptimalControl problem** representing the Bioreactor problem.  
The function defines the state and control variables, boundary conditions, path constraints, growth and light dynamics, and an objective functional.  
It performs direct transcription to create a discretised optimal control problem (DOCP) and the corresponding nonlinear programming (NLP) model.

# Arguments

- `::OptimalControlBackend`: Placeholder type specifying the OptimalControl backend or solver interface.
- `N::Int=500`: (Keyword) Number of discretisation points for the direct transcription grid.

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
    N::Int=steps_number_data(:bioreactor),
    kwargs...,
)

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

    # parameters
    β = 1
    c = 2
    γ = 1
    halfperiod = 5
    Ks = 0.05
    μ2m = 0.1
    μbar = 1
    r = 0.005
    T = final_time_data(:bioreactor)

    # Model
    ocp = @def begin
        t ∈ [0, T], time
        x = (y, s, b) ∈ R³, state
        u ∈ R, control

        x(t) ≥ [0, 0, 0.001]
        0 ≤ u(t) ≤ 1

        0.05 ≤ y(0) ≤ 0.25
        0.5 ≤ s(0) ≤ 5
        0.5 ≤ b(0) ≤ 3

        μ = light(t, halfperiod) * μbar
        μ2 = growth(s(t), μ2m, Ks)

        ẋ(t) == [
            μ * y(t) / (1 + y(t)) - (r + u(t)) * y(t),
            -μ2 * b(t) + u(t) * β * (γ * y(t) - s(t)),
            (μ2 - u(t) * β) * b(t),
        ]

        -∫(μ2 * b(t) / (β + c)) → min
    end

    # initial guess
    init = (state=[50, 50, 50], control=0.5)

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
