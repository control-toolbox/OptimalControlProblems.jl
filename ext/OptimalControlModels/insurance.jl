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

julia> docp, nlp = OptimalControlProblems.insurance(OptimalControlBackend(); N=500);

julia> docp
DOCP object with 500 discretisation points

julia> nlp
NLP model corresponding to the insurance optimisation problem
```

# References

- Problem formulation available at [Bocop repository](https://github.com/control-toolbox/bocop/tree/main/bocop)
"""
function OptimalControlProblems.insurance(::OptimalControlBackend; N::Int=500)

    # parameters
    γ = 0.2
    λ = 0.25
    h0 = 1.5
    w = 1
    s = 10
    k = 0
    σ = 0
    α = 4
    tf = 10

    # Model
    ocp = @def begin
        t ∈ [0, tf], time
        x ∈ R³, state
        u ∈ R⁵, control
        P ∈ R, variable

        #
        I = x[1] # Insurance
        m = x[2] # Expense
        h = u[1]
        R = u[2] # Revenue
        H = u[3] # Health
        U = u[4] # Utility
        dUdR = u[5]

        # constraints
        0 ≤ I(t) ≤ 1.5
        0 ≤ m(t) ≤ 1.5
        0 ≤ h(t) ≤ 25
        0 ≤ R(t) ≤ Inf
        0 ≤ H(t) ≤ Inf
        0 ≤ U(t) ≤ Inf
        0.001 ≤ dUdR(t) ≤ Inf
        0 ≤ P ≤ Inf

        x(0) == [0, 0.001, 0]
        P - x[3](tf) == 0

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
        ẋ(t) == [(1 - γ * t * vprime / dUdR(t)) * h(t), h(t), (1 + σ) * I(t) * fx]

        # objective
        -∫(U(t) * fx) → min
    end

    # Initial guess
    xinit = [0.1, 0.1, 0.1]  # [I, m, x3]
    uinit = [0.1, 0.1, 0.1, 0.1, 0.1]  # [h, R, H, U, dUdR]
    varinit = [0.1]  # [P]
    init = (state=xinit, control=uinit, variable=varinit)

    # DOCP and NLP
    docp = direct_transcription(ocp; init=init, grid_size=N, disc_method=:trapeze)
    nlp = model(docp)

    return docp, nlp
end
