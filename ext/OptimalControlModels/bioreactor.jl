"""
The Bioreactor Problem:
    The problem is formulated as an OptimalControl model and can be found [here](https://github.com/control-toolbox/bocop/tree/main/bocop)
"""
function OptimalControlProblems.bioreactor(::OptimalControlBackend; nh::Int=500, N::Int=30)
    
    # METHANE PROBLEM
    # mu2 according to growth model
    # mu according to light model
    # time scale is [0,10] for 24h (day then night)

    # growth model MONOD
    function growth(s, mu2m, Ks)
        return mu2m * s / (s + Ks)
    end

    # light model: max^2 (0,sin) * mubar
    # DAY/NIGHT CYCLE: [0,2 halfperiod] rescaled to [0,2pi]
    function light(time, halfperiod)
        days = time / (halfperiod * 2)
        tau = (days - floor(days)) * 2π
        return max(0, sin(tau))^2
    end

    # parameters
    beta = 1
    c = 2
    gamma = 1
    halfperiod = 5
    Ks = 0.05
    mu2m = 0.1
    mubar = 1
    r = 0.005
    T = 10N

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

        mu = light(t, halfperiod) * mubar
        mu2 = growth(s(t), mu2m, Ks)

        ẋ(t) == [
            mu * y(t) / (1 + y(t)) - (r + u(t)) * y(t),
            -mu2 * b(t) + u(t) * beta * (gamma * y(t) - s(t)),
            (mu2 - u(t) * beta) * b(t),
        ]

        -∫(mu2 * b(t) / (beta + c)) → min

    end

    # Initial guess
    init = (state=[50, 50, 50], control=0.5)

    # NLPModel + DOCP
    docp = direct_transcription(ocp; init=init, grid_size=nh)
    nlp = model(docp)
    return docp, nlp
end
