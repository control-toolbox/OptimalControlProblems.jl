"""
The Beam Problem:
    The problem is formulated as an OptimalControl model and can be found [here](https://github.com/control-toolbox/bocop/tree/main/bocop)
"""
function OptimalControlProblems.beam(::OptimalControlBackend; N::Int=500)

    # model
    ocp = @def begin
        t ∈ [0, 1], time
        x ∈ R², state
        u ∈ R, control
        x(0) == [0, 1]
        x(1) == [0, -1]
        ẋ(t) == [x₂(t), u(t)]
        0 ≤ x₁(t) ≤ 0.1
        -10 ≤ u(t) ≤ 5
        ∫(u(t)^2) → min
    end

    # initial guess
    init = (state=[0.05, 0.1], control=0.1)

    # DOCP and NLP
    docp = direct_transcription(ocp; init=init, grid_size=N, disc_method=:trapeze)
    nlp = model(docp)
    return docp, nlp
end
