"""
The Van der Pol Problem:
    The problem is formulated as an OptimalControl model and can be found [here](https://github.com/control-toolbox/bocop/tree/main/bocop)
"""
function OptimalControlProblems.vanderpol(::OptimalControlBackend; nh::Int=500)

    # parameters
    ω = 1
    ε = 1
    tf = 2

    # model
    ocp = @def begin
        t ∈ [0, tf], time
        x ∈ R², state
        u ∈ R, control

        x(0) == [1, 0]

        ẋ(t) == [x[2](t), ε * ω * (1 - x[1](t)^2) * x[2](t) - ω^2 * x[1](t) + u(t)]

        0.5∫(x[1](t)^2 + x[2](t)^2 + u(t)^2) → min
    end

    # initial guess
    xinit = [0.1, 0.1]  # [x1, x2]
    uinit = [0.1]  # [u]
    init = (state=xinit, control=uinit)

    # DOCP and NLP
    docp = direct_transcription(ocp; init=init, grid_size=nh, disc_method=:trapeze)
    nlp = model(docp)

    return docp, nlp
end
