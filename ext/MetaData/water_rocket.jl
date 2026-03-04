water_rocket_meta = OrderedDict(
    :grid_size => 100,
    :parameters => (
        g = 9.80665,
        rho_w = 1000.0,
        p_a = 1.01325e5,
        k = 1.2,
        V_b = 2.0e-3,
        A_out = pi * (0.013)^2 / 4,
        S = pi * (0.106)^2 / 4,
        C_d = 0.345,
        rho_a = 1.225,
        m_empty = 0.15,
        t0 = 0.0,
        r_t0 = 0.0,
        h_t0 = 0.0,
        v_t0 = 0.1,
        p_t0 = 6.5e5 + 1.01325e5,
        tf_start = 0.3,
        Vw0_start = 1.0e-3,
        gamma0_start = 0.785, # pi/4 approx
    ),
)
