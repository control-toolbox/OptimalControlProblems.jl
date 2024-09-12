"""
The Bioreactor Problem:
    The problem is formulated as an OptimalControl model and can be found [here](https://github.com/control-toolbox/bocop/tree/main/bocop)
"""
function OptimalControlProblems.bioreactor(::OptimalControlBackend; nh::Int=100, N::Int=30)
    # constants
    beta = 1
    c = 2
    gamma = 1
    halfperiod = 5
    Ks = 0.05
    mu2m = 0.1
    mubar = 1
    r = 0.005
    T = 10 * N

    # Model
    @def ocp begin
        # constants
        beta = 1
        c = 2
        gamma = 1
        halfperiod = 5
        Ks = 0.05
        mu2m = 0.1
        mubar = 1
        r = 0.005
        T = 10 * N

        # ocp
        t ∈ [0, T], time
        x ∈ R³, state
        u ∈ R, control
        y = x[1]
        s = x[2]
        b = x[3]
        mu2 = mu2m * s(t) / (s(t) + Ks)
        [0, 0, 0.001] ≤ x(t) ≤ [Inf, Inf, Inf]
        0 ≤ u(t) ≤ 1
        0.05 ≤ y(0) ≤ 0.25
        0.5 ≤ s(0) ≤ 5
        0.5 ≤ b(0) ≤ 3

        # dynamics
        ẋ(t) == dynamics(t, x(t), u(t))

        # objective
        ∫(b(t) / (beta + c)) → max
    end

    # dynamics
    function dynamics(t, x, u)
        y, s, b = x
        pi = 3.141592653589793
        days = t / (halfperiod * 2)
        tau = (days - floor(days)) * 2 * pi
        light = max(0, sin(tau))^2
        mu = light * mubar
        mu2 = mu2m * s / (s + Ks)
        return [
            mu * y / (1 + y) - (r + u) * y,
            -mu2 * b + u * beta * (gamma * y - s),
            (mu2 - u * beta) * b,
        ]
    end

    # Initial guess
    init = (state=[50, 50, 50], control=0.5)

    # NLPModel
    nlp = direct_transcription(ocp; init=init, grid_size=nh)[2]

    return nlp
end
