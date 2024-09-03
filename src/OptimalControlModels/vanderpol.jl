"""
The Van der Pol Problem:
    The problem is formulated as an OptimalControl modeln and can be found [here](https://github.com/control-toolbox/bocop/tree/main/bocop)
"""
function vanderpol(;nh::Int=100)
    @def ocp begin
        # constants
        omega = 1
        epsilon = 1

        t ∈ [0, 2], time
        x ∈ R², state
        u ∈ R, control
        x(0) == [1, 0]
        ẋ(t) == [x[2](t), epsilon * omega * (1 - x[1](t)^2) * x[2](t) - omega^2 * x[1](t) + u(t)]
        ∫(0.5 * (x[1](t)^2 + x[2](t)^2 + u(t)^2)) → min
    end

    # Initial guess
    function vanderpol_init(;nh)
        return ()
    end
    init = vanderpol_init(;nh=nh)

    # NLPModel
    nlp = direct_transcription(ocp ,init = init, grid_size = nh)[2]

    return nlp


end