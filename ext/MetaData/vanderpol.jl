vanderpol_meta = OrderedDict(
    :grid_size => 500,
    :parameters => (t0=0, tf=2, ω=1, ε=1, x₁_t0=1, x₂_t0=0),
    :status => (
        OptimalControl=:valid,
        OptimalControlScalarized=:valid,
        JuMP=:valid,
        ModelingToolkit=:valid,
    ),
    # initial guess of every modeling (#377), from the parameters
    :init => p -> (state=[0.1, 0.1], control=0.1),
)
