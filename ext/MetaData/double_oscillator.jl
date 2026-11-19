double_oscillator_meta = OrderedDict(
    :grid_size => 500,
    :parameters => (
        t0=0,
        tf=2π,
        m1=100,    # [kg]
        m2=2,      # [kg]
        c=0.5,     # [Ns/m]
        k1=100,    # [N/m]
        k2=3,      # [N/m]
        u_l=-1,
        u_u=1,
        x₁_t0=0,
        x₂_t0=0,
    ),
    :parameter_info => (
        t0=(unit="s", meaning="initial time"),
        tf=(unit="s", meaning="final time, one period of the force F"),
        m1=(unit="kg", meaning="mass of the first body, driven by the force F"),
        m2=(unit="kg", meaning="mass of the second body, damped by the control"),
        c=(unit="N·s/m", meaning="damping coefficient of the second body"),
        k1=(unit="N/m", meaning="stiffness of the spring between the first body and the frame"),
        k2=(unit="N/m", meaning="stiffness of the spring between the two bodies"),
        u_l=(unit=nothing, meaning="lower bound of the control u"),
        u_u=(unit=nothing, meaning="upper bound of the control u"),
        x₁_t0=(unit="m", meaning="initial position x₁ of the first body"),
        x₂_t0=(unit="m", meaning="initial position x₂ of the second body"),
    ),
    :status => (OptimalControl=:valid, OptimalControlScalarized=:valid, JuMP=:valid, ModelingToolkit=:valid),
    # initial guess of every modeling (#377), from the parameters
    :init => p -> (state=[0.1, 0.1, 0.1, 0.1], control=0.1),
)
