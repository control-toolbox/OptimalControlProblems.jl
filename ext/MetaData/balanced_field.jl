balanced_field_meta = OrderedDict(
    :grid_size => 200,
    :parameters => (
        t0=0.0,
        m=79015.8,      # mass (kg)
        g=9.80665,      # gravity (m/s^2)
        ρ=1.225,        # air density (kg/m^3)
        S=124.7,        # surface (m^2)
        CD0=0.03,       # zero-lift drag coefficient
        AR=9.45,        # aspect ratio
        e=0.801,        # Oswald efficiency
        CL0=0.5,        # zero-alpha lift coefficient
        CL_max=2.0,     # max lift coefficient
        h_w=1.0,        # height of wing above CoG (m)
        span=35.7,      # wingspan (m)
        α_max=0.1745,   # alpha at CL_max (10 degrees in rad)
        T=120102.0,     # thrust (N) - one engine out
        r_t0=600.0,     # initial range (m) at V1
        v_t0=70.0,      # initial velocity (m/s) at V1
        h_t0=0.0,       # initial altitude (m)
        γ_t0=0.0,       # initial flight path angle (rad)
        h_tf=10.668,    # final altitude (35 ft in m)
        γ_tf=0.0873,    # final flight path angle (5 degrees in rad)
        v_min=0.0,
        v_max=100.0,
        α_min=-0.1745,  # -10 degrees
        α_max_ctrl=0.2618, # 15 degrees
    ),
    :parameter_info => (
        t0=(unit="s", meaning="initial time, at the speed V1"),
        m=(unit="kg", meaning="mass of the aircraft"),
        g=(unit="m/s²", meaning="gravitational acceleration"),
        ρ=(unit="kg/m³", meaning="air density"),
        S=(unit="m²", meaning="wing area"),
        CD0=(unit=nothing, meaning="zero-lift drag coefficient"),
        AR=(unit=nothing, meaning="aspect ratio of the wing"),
        e=(unit=nothing, meaning="Oswald efficiency factor"),
        CL0=(unit=nothing, meaning="lift coefficient at zero angle of attack"),
        CL_max=(unit=nothing, meaning="maximum lift coefficient, reached at α_max"),
        h_w=(
            unit="m",
            meaning="height of the wing above the centre of gravity, in the ground effect",
        ),
        span=(unit="m", meaning="wingspan"),
        α_max=(unit="rad", meaning="angle of attack at the maximum lift coefficient"),
        T=(unit="N", meaning="thrust, one engine out"),
        r_t0=(unit="m", meaning="initial range, at V1"),
        v_t0=(unit="m/s", meaning="initial velocity, V1"),
        h_t0=(unit="m", meaning="initial altitude"),
        γ_t0=(unit="rad", meaning="initial flight path angle"),
        h_tf=(unit="m", meaning="final altitude, the screen height of 35 ft"),
        γ_tf=(unit="rad", meaning="final flight path angle, 5°"),
        v_min=(
            unit="m/s",
            meaning="lower bound of the velocity (not used by the models yet, which use 1)",
        ),
        v_max=(
            unit="m/s",
            meaning="upper bound of the velocity (not used by the models yet)",
        ),
        α_min=(
            unit="rad",
            meaning="lower bound of the angle of attack (not used by the models yet, which use 0)",
        ),
        α_max_ctrl=(unit="rad", meaning="upper bound of the angle of attack"),
    ),
    :status => (OptimalControl=:valid, OptimalControlScalarized=:valid, JuMP=:valid),
    # initial guess of every modeling (#377), from the parameters
    :init =>
        p -> (
            state=[p.r_t0, p.v_t0 + 10.0, p.h_tf / 2.0, p.γ_tf],
            control=0.1,
            variable=10.0,
        ),
)
