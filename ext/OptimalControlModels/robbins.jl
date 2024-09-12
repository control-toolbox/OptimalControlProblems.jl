"""
The Robbins Problem:
    The problem is formulated as an OptimalControl model and can be found [here](https://github.com/control-toolbox/bocop/tree/main/bocop)
"""
function OptimalControlProblems.robbins(::OptimalControlBackend; nh::Int=100, N::Int=30)
    # Model
    ocp = @def begin
        # constants
        alpha = 3
        beta = 0
        gamma = 0.5

        t ∈ [0, 10], time
        x ∈ R³, state
        u ∈ R, control
        0 ≤ x[1](t) ≤ Inf
        x(0) == [1, -2, 0]
        x(10) == [0, 0, 0]
        ẋ(t) == [x[2](t), x[3](t), u(t)]
        ∫(alpha * x[1](t) + beta * x[1](t)^2 + gamma * u(t)^2) → min
    end

    # Initial guess
    init = ()

    # NLPModel
    nlp = direct_transcription(ocp; init=init, grid_size=nh)[2]

    return nlp
end
