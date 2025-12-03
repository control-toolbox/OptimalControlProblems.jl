"""
$(TYPEDSIGNATURES)

Constructs an **OptimalControl problem** for an insurance system optimisation.  
The function defines state and control variables, system dynamics, constraints on insurance, expenses, revenue, health, utility, and auxiliary variables, and sets up a cost functional aimed at maximising expected utility over time.  
It returns both a discretised direct optimal control problem (DOCP) and the corresponding nonlinear programming (NLP) model.

# Arguments

- `::OptimalControlBackend`: Placeholder type specifying the OptimalControl backend or solver interface.
- `grid_size::Int=500`: (Keyword) Number of discretisation points for the direct transcription grid.

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
function OptimalControlProblems.insurance(
    ::OptimalControlBackend,
    description::Symbol...;
    grid_size::Int=grid_size_data(:insurance),
    parameters::Union{Nothing,NamedTuple}=nothing,
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
    I_l = params[:I_l]
    I_u = params[:I_u]
    m_l = params[:m_l]
    m_u = params[:m_u]
    h_l = params[:h_l]
    h_u = params[:h_u]
    R_l = params[:R_l]
    H_l = params[:H_l]
    U_l = params[:U_l]
    dUdR_l = params[:dUdR_l]
    P_l = params[:P_l]
    I_t0 = params[:I_t0]
    m_t0 = params[:m_t0]
    x₃_t0 = params[:x₃_t0]

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
        I_l ≤ I(t) ≤ I_u
        m_l ≤ m(t) ≤ m_u
        h_l ≤ h(t) ≤ h_u
        R(t) ≥ R_l
        H(t) ≥ H_l
        U(t) ≥ U_l
        dUdR(t) ≥ dUdR_l
        P ≥ P_l

        x(t0) == [I_t0, m_t0, x₃_t0]
        P - x₃(tf) == 0

        #
        ε = k * (t - t0) / (tf - t + 1)

        # illness distribution
        fx = λ * exp(-λ * (t - t0)) + exp(-λ * (tf - t0)) / (tf - t0)

        # expense effect
        v = m(t)^(α / 2) / (1 + m(t)^(α / 2))
        vprime = α / 2 * m(t)^(α / 2 - 1) / (1 + m(t)^(α / 2))^2

        # constraints
        R(t) - (w - P + I(t) - m(t) - ε) == 0
        H(t) - (h0 - γ * (t - t0) * (1 - v)) == 0
        U(t) - (1 - exp(-s * R(t)) + H(t)) == 0
        dUdR(t) - (s * exp(-s * R(t))) == 0

        # dynamics
        ẋ(t) == [(1 - γ * (t - t0) * vprime / dUdR(t)) * h(t), h(t), (1 + σ) * I(t) * fx]

        # objective
        ∫(U(t) * fx) → min
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
