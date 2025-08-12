"""
The electric Vehicle Problem
    Implement optimal control of an electric vehicle.
    The problem is formulated as an OptimalControl model.
Ref: [PS2011] Nicolas Petit and Antonio Sciarretta. "Optimal drive of electric vehicles using an inversion-based trajectory generation approach." IFAC Proceedings Volumes 44, no. 1 (2011): 14519-14526.
"""
function OptimalControlProblems.electric_vehicle(::OptimalControlBackend; nh=500)

    # parameters
    D = 10
    tf = 1
    b1 = 1e0
    b2 = 1e0
    h0 = 0.1
    h1 = 1
    h2 = 1e-3
    α0, α1, α2, α3 = (3, 0.4, -1, 0.1)

    # model
    ocp = @def begin
        t ∈ [0, tf], time
        y = (x, v) ∈ R², state
        u ∈ R, control

        x(0) == 0, (x_i)
        v(0) == 0, (v_i)
        x(tf) == D, (x_f)
        v(tf) == 0, (v_f)

        ẏ(t) == dynamics(x(t), v(t), u(t))

        ∫(b1 * u(t) * v(t) + b2 * u(t)^2) → min
    end

    road(x) = α0 + α1 * x + α2 * x^2 + α3 * x^3
    dynamics(x, v, u) = [v, h1 * u - h2 * v^2 - h0 - road(x)]

    # initial guess
    yinit = [0.1, 0.1]  # [x, v]
    uinit = [0.1]       # [u]
    init = (state=yinit, control=uinit)

    # DOCP and NLP
    docp = direct_transcription(ocp; init=init, grid_size=nh, disc_method=:trapeze)
    nlp = model(docp)

    return docp, nlp
end
