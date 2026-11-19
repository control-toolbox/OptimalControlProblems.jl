cart_pendulum_meta = OrderedDict(
    :grid_size => 500,
    :parameters => (
        t0=0,
        g=9.81,            # gravitation [m/s^2]
        L=1,               # pendulum length [m]
        m=1,               # pendulum mass [kg]
        mcart=0.5,         # cart mass [kg]
        Fex_l=-5,
        Fex_u=5,
        x_l=-1,
        x_u=1,
        v_l=-2,
        v_u=2,
        tf_l=0.1,
        x_t0=0,
        θ_t0=0,
        ω_t0=0,
        θ_tf=π,
        ω_tf=0,
    ),
    :parameter_info => (
        t0=(unit="s", meaning="initial time"),
        g=(unit="m/s²", meaning="gravitational acceleration"),
        L=(unit="m", meaning="length of the pendulum"),
        m=(unit="kg", meaning="mass of the pendulum"),
        mcart=(unit="kg", meaning="mass of the cart"),
        Fex_l=(unit="N", meaning="lower bound of the force Fex on the cart"),
        Fex_u=(unit="N", meaning="upper bound of the force Fex on the cart"),
        x_l=(unit="m", meaning="lower bound of the position of the cart"),
        x_u=(unit="m", meaning="upper bound of the position of the cart"),
        v_l=(unit="m/s", meaning="lower bound of the velocity of the cart"),
        v_u=(unit="m/s", meaning="upper bound of the velocity of the cart"),
        tf_l=(unit="s", meaning="lower bound of the final time"),
        x_t0=(unit="m", meaning="initial position of the cart"),
        θ_t0=(unit="rad", meaning="initial angle of the pendulum"),
        ω_t0=(unit="rad/s", meaning="initial angular velocity of the pendulum"),
        θ_tf=(unit="rad", meaning="final angle of the pendulum (upright)"),
        ω_tf=(unit="rad/s", meaning="final angular velocity of the pendulum"),
    ),
    :status => (OptimalControl=:valid, JuMP=:valid),
    # initial guess of every modeling (#377), from the parameters
    :init => p -> (state=[0.1, 0.1, 0.1, 0.1], control=0.1, variable=[1.0, 0.1]),
)
