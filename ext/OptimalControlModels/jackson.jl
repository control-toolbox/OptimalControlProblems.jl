"""
The Jackson Problem:
    The problem is formulated as an OptimalControl model and can be found [here](https://github.com/control-toolbox/bocop/tree/main/bocop)
"""
function OptimalControlProblems.jackson(::OptimalControlBackend; nh::Int=500)

    # parameters
    k1 = 1
    k2 = 10
    k3 = 1
    tf = 4

    # model
    ocp = @def begin

        t ∈ [0, tf], time
        x ∈ R³, state
        u ∈ R, control

        a = x[1]
        b = x[2]

        x(0) == [1, 0, 0]

        [0, 0, 0] ≤ x(t) ≤ [1.1, 1.1, 1.1]
        0 ≤ u(t) ≤ 1
        
        ẋ(t) == [
            -u(t) * (k1 * a(t) - k2 * b(t)),
            u(t) * (k1 * a(t) - k2 * b(t)) - (1 - u(t)) * k3 * b(t),
            (1 - u(t)) * k3 * b(t),
        ]

        -x[3](tf) → min

    end

    # initial guess
    xinit = [0.1, 0.1, 0.1]  # [a, b, x3]
    uinit = [0.1]  # [u]
    init = (state=xinit, control=uinit)

    # DOCP and NLP
    docp = direct_transcription(ocp; init=init, grid_size=nh)
    nlp = model(docp)

    return docp, nlp
end
