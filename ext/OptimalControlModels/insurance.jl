"""
The Insurance Problem:
    The problem is formulated as an OptimalControl model and can be found [here](https://github.com/control-toolbox/bocop/tree/main/bocop)
"""
function OptimalControlProblems.insurance(::OptimalControlBackend; nh::Int=100)
    # constants
    gamma = 0.2
    lambda = 0.25
    h0 = 1.5
    w = 1
    s = 10
    k = 0
    sigma = 0
    alpha = 4
    tf = 10

    # Model
    ocp = @def begin
        # constants
        gamma = 0.2
        lambda = 0.25
        h0 = 1.5
        w = 1
        s = 10
        k = 0
        sigma = 0
        alpha = 4
        tf = 10

        t ∈ [0, tf], time
        x ∈ R³, state
        u ∈ R⁵, control
        P ∈ R, variable
        I = x[1] # Insurance
        m = x[2] # Expense
        h = u[1]
        R = u[2] # Revenue
        H = u[3] # Health
        U = u[4] # Utility
        dUdR = u[5]

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

        epsilon = k * t / (tf - t + 1)
        # illness distribution
        fx = lambda * exp(-lambda * t) + exp(-lambda * tf) / tf
        # expense effect
        v = m(t)^(alpha / 2) / (1 + m(t)^(alpha / 2))
        vprime = alpha / 2 * m(t)^(alpha / 2 - 1) / (1 + m(t)^(alpha / 2))^2

        R(t) - (w - P + I(t) - m(t) - epsilon) == 0
        H(t) - (h0 - gamma * t * (1 - v)) == 0
        U(t) - (1 - exp(-s * R(t)) + H(t)) == 0
        dUdR(t) - (s * exp(-s * R(t))) == 0

        # dynamics
        ẋ(t) == [(1 - gamma * t * vprime / dUdR(t)) * h(t), h(t), (1 + sigma) * I(t) * fx]

        # objective
        ∫(U(t) * fx) → max
    end

    # Initial guess
    init = ()

    # NLPModel
    nlp = direct_transcription(ocp; init=init, grid_size=nh)[2]

    return nlp
end
