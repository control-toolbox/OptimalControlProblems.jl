"""
The Jackson Problem:
    The problem is formulated as an OptimalControl model and can be found [here](https://github.com/control-toolbox/bocop/tree/main/bocop)
"""
function OptimalControlProblems.jackson(::OptimalControlBackend; nh::Int=100, N::Int=30)
    # Model
    ocp = @def begin
        # constants
        k1 = 1
        k2 = 10
        k3 = 1

        t ∈ [0, 4], time
        x ∈ R³, state
        u ∈ R, control
        [0, 0, 0] ≤ x(t) ≤ [1.1, 1.1, 1.1]
        0 ≤ u(t) ≤ 1
        x(0) == [1, 0, 0]
        a = x[1]
        b = x[2]
        ẋ(t) == [
            -u(t) * (k1 * a(t) - k2 * b(t)),
            u(t) * (k1 * a(t) - k2 * b(t)) - (1 - u(t)) * k3 * b(t),
            (1 - u(t)) * k3 * b(t),
        ]
        x[3](4) → max
    end

    # Initial guess
    init = ()

    # NLPModel + DOCP
    res = direct_transcription(ocp; init=init, grid_size=nh)

    return res
end
