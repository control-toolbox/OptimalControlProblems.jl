"""
The Hanging Chain Problem:
    We want to find the shape of a chain hanging between two points a and b, with a length L.
    The objective is to minimize the potential energy of the chain.
    The problem is formulated as an OptimalControl model.
"""
function OptimalControlProblems.chain(::OptimalControlBackend; nh::Int=100)
    # parameters
    L = 4
    a = 1
    b = 3
    tf = 1.0

    # Model
    ocp = @def begin
        ## parameters
        L = 4
        a = 1
        b = 3
        tf = 1.0

        ## define the Problem
        t ∈ [0.0, tf], time
        x ∈ R³, state
        u ∈ R¹, control

        ## constraints
        # initial conditions
        x₁(0.0) == a, (x1_ic)
        x₂(0.0) == 0.0, (x2_ic)
        x₃(0.0) == 0.0, (x3_ic)
        # final conditions
        x₁(tf) == b, (x1_con)
        x₃(tf) == L, (x3_con)

        ## dynamics
        ẋ(t) == dynamics(x(t), u(t))

        ## objective
        x₂(tf) → min
    end

    # dynamics
    function dynamics(x, u)
        return [u, x[1] * sqrt(1 + u^2), sqrt(1 + u^2)]
    end

    # Initial guess
    tmin = b > a ? 1 / 4 : 3 / 4
    xinit =
        t -> [
            4 * abs(b - a) * t / tf * (1 / 2 * t / tf - tmin) + a,
            (4 * abs(b - a) * t / tf * (1 / 2 * t / tf - tmin) + a) *
            (4 * abs(b - a) * (t / tf - tmin)),
            4 * abs(b - a) * (t / tf - tmin),
        ]
    uinit = t -> 4 * abs(b - a) * (t / tf - tmin)
    init = (state=xinit, control=uinit)

    # NLPModel
    nlp = direct_transcription(ocp; init=init, grid_size=nh)[2]

    return nlp
end
