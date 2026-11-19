dielectrophoretic_particle_meta = OrderedDict(
    :grid_size => 500,
    :parameters => (t0=0, x_t0=1, y_t0=0, x_tf=2, α=-0.75, c=1, u_l=-1, u_u=1, tf_l=0.1),
    :parameter_info => (
        t0=(unit=nothing, meaning="initial time"),
        x_t0=(unit=nothing, meaning="initial position x of the particle"),
        y_t0=(unit=nothing, meaning="initial value of y"),
        x_tf=(unit=nothing, meaning="final position x of the particle"),
        α=(
            unit=nothing,
            meaning="coefficient of the quadratic term α u² in the dynamics of x",
        ),
        c=(unit=nothing, meaning="damping coefficient in the dynamics of y"),
        u_l=(unit=nothing, meaning="lower bound of the control u"),
        u_u=(unit=nothing, meaning="upper bound of the control u"),
        tf_l=(unit=nothing, meaning="lower bound of the final time"),
    ),
    :status => (OptimalControl=:valid, OptimalControlScalarized=:valid, JuMP=:valid),
    # initial guess of every modeling (#377), from the parameters
    :init => p -> (state=[1, 0.5], control=0.5, variable=8),
)
