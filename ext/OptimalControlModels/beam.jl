"""
The Beam Problem:
    The problem is formulated as an OptimalControl model and can be found [here](https://github.com/control-toolbox/bocop/tree/main/bocop)
"""
function OptimalControlProblems.beam(::OptimalControlBackend; nh::Int=100)
    # Model
    ocp = @def begin
        t ∈ [0, 1], time
        x ∈ R², state
        u ∈ R, control
        x(0) == [0, 1]
        x(1) == [0, -1]
        ẋ(t) == [x₂(t), u(t)]
        0 ≤ x₁(t) ≤ 0.1
        -10 ≤ u(t) ≤ 10
        ∫(u(t)^2) → min
    end

    # Initial guess
    init = (state = [0.0, 0.0], control = 0.0,)

    # NLPModel
    nlp = direct_transcription(ocp ,init = init, grid_size = nh)[2]

    return nlp

end