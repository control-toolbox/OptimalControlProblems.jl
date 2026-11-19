rocket_meta = OrderedDict(
    :grid_size => 500,
    :parameters =>
        (t0=0, h_t0=1, v_t0=0, m_t0=1, g0=1, Tc=3.5, hc=500, vc=620, mc=0.6, T_l=0, tf_l=0),
    :parameter_info => (
        t0=(unit=nothing, meaning="initial time"),
        h_t0=(unit=nothing, meaning="initial altitude, and lower bound of the altitude"),
        v_t0=(unit=nothing, meaning="initial velocity, and lower bound of the velocity"),
        m_t0=(unit=nothing, meaning="initial mass, and upper bound of the mass"),
        g0=(unit=nothing, meaning="gravitational acceleration at the initial altitude"),
        Tc=(unit=nothing, meaning="maximum thrust, in units of the initial weight m_t0 g0"),
        hc=(unit=nothing, meaning="scale height of the atmosphere in the drag"),
        vc=(unit=nothing, meaning="drag coefficient factor, Dc = vc m_t0 / (2 g0)"),
        mc=(unit=nothing, meaning="final mass, as a fraction of the initial mass"),
        T_l=(unit=nothing, meaning="lower bound of the thrust T"),
        tf_l=(unit=nothing, meaning="lower bound of the final time"),
    ),
    :status => (OptimalControl=:valid, OptimalControlScalarized=:valid, JuMP=:valid),
    # initial guess of every modeling (#377), from the parameters
    :init =>
        p -> begin
            (; m_t0, mc, Tc, g0) = p
            tf = 1
            m_tf = mc * m_t0
            Tmax = Tc * m_t0 * g0
            return (
                state=t ->
                    [1, (t / tf) * (1 - t / tf), (m_tf - m_t0) * (t / tf) + m_t0],
                control=Tmax / 2,
                variable=tf,
            )
        end,
)
