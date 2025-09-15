"""
$(TYPEDSIGNATURES)

Constructs an **OptimalControl problem** for an insurance system optimisation.  
The function defines state and control variables, system dynamics, constraints on insurance, expenses, revenue, health, utility, and auxiliary variables, and sets up a cost functional aimed at maximising expected utility over time.  
It returns both a discretised direct optimal control problem (DOCP) and the corresponding nonlinear programming (NLP) model.

# Arguments

- `::OptimalControlBackend`: Placeholder type specifying the OptimalControl backend or solver interface.
- `N::Int=500`: (Keyword) Number of discretisation points for the direct transcription grid.

# Returns

- `docp`: The direct optimal control problem object representing the insurance optimisation problem.
- `nlp`: The corresponding nonlinear programming model obtained from the DOCP, suitable for numerical optimisation.

# Example

```julia-repl
julia> using OptimalControlProblems

julia> docp = OptimalControlProblems.insurance(OptimalControlBackend(); N=500);
```

# References

- Problem formulation available at [Bocop repository](https://github.com/control-toolbox/bocop/tree/main/bocop)
"""
function OptimalControlProblems.insurance_s(
    ::OptimalControlBackend,
    description::Symbol...;
    grid_size::Int=steps_number_data(:insurance),
    parameters::Union{Nothing, NamedTuple}=nothing,
    kwargs...,
)

    # parameters
    params = parameters_data(:insurance, parameters)
    t0 = params[:t0]
    tf = params[:tf]
    γ = params[:γ]
    λ = params[:λ]
    h0 = params[:h0]
    w = params[:w]
    s = params[:s]
    k = params[:k]
    σ = params[:σ]
    α = params[:α]

    # I: Insurance
    # m: Expense
    # R: Revenue
    # H: Health
    # U: Utility

    # Model
    ocp = @def begin
        P ∈ R, variable
        t ∈ [t0, tf], time
        x = (I, m, x₃) ∈ R³, state
        u = (h, R, H, U, dUdR) ∈ R⁵, control

        # constraints
        0 ≤ I(t) ≤ 1.5
        0 ≤ m(t) ≤ 1.5
        0 ≤ h(t) ≤ 25
        0 ≤ R(t) ≤ Inf
        0 ≤ H(t) ≤ Inf
        0 ≤ U(t) ≤ Inf
        0.001 ≤ dUdR(t) ≤ Inf
        0 ≤ P ≤ Inf

        x(t0) == [0, 0.001, 0]
        P - x₃(tf) == 0

        ε = k * t / (tf - t + 1)

        # illness distribution
        fx = λ * exp(-λ * t) + exp(-λ * tf) / tf

        # expense effect
        v = m(t)^(α / 2) / (1 + m(t)^(α / 2))
        vprime = α / 2 * m(t)^(α / 2 - 1) / (1 + m(t)^(α / 2))^2

        # constraints
        R(t) - (w - P + I(t) - m(t) - ε) == 0
        H(t) - (h0 - γ * t * (1 - v)) == 0
        U(t) - (1 - exp(-s * R(t)) + H(t)) == 0
        dUdR(t) - (s * exp(-s * R(t))) == 0

        # dynamics
        ∂(I)(t) == (1 - γ * t * vprime / dUdR(t)) * h(t)
        ∂(m)(t) == h(t)
        ∂(x₃)(t) == (1 + σ) * I(t) * fx

        # objective
        -∫(U(t) * fx) → min
    end

    # initial guess
    xinit = [0.1, 0.1, 0.1]  # [I, m, x₃]
    uinit = [0.1, 0.1, 0.1, 0.1, 0.1]  # [h, R, H, U, dUdR]
    varinit = [0.1]  # [P]
    init = (state=xinit, control=uinit, variable=varinit)

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
