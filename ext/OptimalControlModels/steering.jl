"""
Particle Steering Problem:
    We want to find the optimal trajectory of a particle.
    The objective is to minimize the time taken to achieve a given altitude and terminal velocity.
    The problem is formulated as an OptimalControl model.
"""
function OptimalControlProblems.steering(::OptimalControlBackend; nh::Int=100)
    # parameters
    a = 100.0
    u_min = -pi / 2.0
    u_max = pi / 2.0
    xs = zeros(4)
    xf = [NaN, 5.0, 45.0, 0.0]

    # Model
    ocp = @def begin
        # parameters
        a = 100.0
        u_min = -pi / 2.0
        u_max = pi / 2.0
        xs = zeros(4)
        xf = [NaN, 5.0, 45.0, 0.0]

        ## define the Problem
        tf ∈ R, variable
        t ∈ [0.0, tf], time
        x ∈ R⁴, state
        u ∈ R¹, control

        ## constraints
        # time constraints
        tf ≥ 0.0, (tf_con)
        # initial conditions
        x(0.0) == xs, (x_ic)
        # final conditions
        x(tf) == xf, (x_fc)
        # control constraints
        u_min ≤ u(t) ≤ u_max, (u_con)

        ## dynamics
        ẋ(t) == dynamics(x(t), u(t))

        ## objective
        tf → min
    end

    # dynamics
    function dynamics(x, u)
        return [x[3], x[4], a * cos(u), a * sin(u)]
    end

    # Initial guess
    function gen_x0(t, i)
        if i == 1 || i == 4
            return 0.0
        elseif i == 2
            return 5 * t
        elseif i == 3
            return 45.0 * t
        end
    end
    xinit = t -> [gen_x0(t, i) for i in 1:4]
    init = (state=xinit, control=0.0, variable=1.0)

    # NLPModel + DOCP
    docp, nlp = direct_transcription(ocp; init=init, grid_size=nh)

    return docp, nlp
end
