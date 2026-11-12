rocket_meta = OrderedDict(
    :grid_size => 500,
    :parameters =>
        (t0=0, h_t0=1, v_t0=0, m_t0=1, g0=1, Tc=3.5, hc=500, vc=620, mc=0.6, T_l=0, tf_l=0),
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
