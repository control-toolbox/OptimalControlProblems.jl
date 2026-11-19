electric_vehicle_meta = OrderedDict(
    :grid_size => 500,
    :parameters => (
        t0=0,
        tf=1,
        b1=1e0,
        b2=1e0,
        h0=0.1,
        h1=1,
        h2=1e-3,
        α0=3,
        α1=0.4,
        α2=-1,
        α3=0.1,
        x_t0=0,
        v_t0=0,
        x_tf=10,
        v_tf=0,
    ),
    :status => (OptimalControl=:valid, OptimalControlScalarized=:valid, JuMP=:valid, ModelingToolkit=:valid),
    # initial guess of every modeling (#377), from the parameters
    :init => p -> (state=[0.1, 0.1], control=0.1),
)
