"""The **insurance problem** is a benchmark in optimal control inspired by health and financial economics.

Builds the `insurance` problem with the OptimalControl backend.

```julia-repl
julia> using OptimalControlProblems
julia> insurance(OptimalControlBackend())
```

Default parameters and the modeling status are available through
`problem_metadata(:insurance)` and `OptimalControlProblems.status(:insurance)`.
"""
function OptimalControlProblems.insurance(
    ::OptimalControlBackend; parameters::Union{Nothing,NamedTuple}=nothing, init=nothing
)
    OptimalControlProblems.Catalogue._warn_if_draft(:insurance, :OptimalControl)

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
        -∫(U(t) * fx) → min
    end

    # initial guess: from the metadata (#377), unless given
    init = OptimalControlProblems.Metadata._initial_guess(:insurance, params, init)

    return OptimalControlProblems.Problems._problem(
        :insurance, :OptimalControl, ocp, init, params
    )
end
