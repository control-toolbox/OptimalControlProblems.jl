steering_meta = OrderedDict(
    :grid_size => 500,
    :parameters => (
        t0=0,
        a=100,
        u_min=(-π / 2),
        u_max=π / 2,
        tf_l=0,
        x₁_t0=0,
        x₂_t0=0,
        x₃_t0=0,
        x₄_t0=0,
        x₂_tf=5,
        x₃_tf=45,
        x₄_tf=0,
    ),
    :parameter_info => (
        t0=(unit=nothing, meaning="initial time"),
        a=(unit=nothing, meaning="magnitude of the acceleration"),
        u_min=(unit="rad", meaning="lower bound of the steering angle u"),
        u_max=(unit="rad", meaning="upper bound of the steering angle u"),
        tf_l=(unit=nothing, meaning="lower bound of the final time"),
        x₁_t0=(unit=nothing, meaning="initial position x₁"),
        x₂_t0=(unit=nothing, meaning="initial position x₂"),
        x₃_t0=(unit=nothing, meaning="initial velocity x₃"),
        x₄_t0=(unit=nothing, meaning="initial velocity x₄"),
        x₂_tf=(unit=nothing, meaning="final position x₂"),
        x₃_tf=(unit=nothing, meaning="final velocity x₃"),
        x₄_tf=(unit=nothing, meaning="final velocity x₄"),
    ),
    :status => (OptimalControl=:valid, OptimalControlScalarized=:valid, JuMP=:valid),
    # initial guess of every modeling (#377), from the parameters
    :init =>
        p -> begin
            t0 = p.t0
            return (
                state=t -> [0.0, 5.0 * (t - t0), 45.0 * (t - t0), 0.0],
                control=0,
                variable=0.6,
            )
        end,
)
