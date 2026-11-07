robbins_meta = OrderedDict(
    :grid_size => 500,
    :parameters => (
        t0=0,
        tf=10,
        α=3,
        β=0,
        γ=0.5,
        x₁_l=0,
        x₁_t0=1,
        x₂_t0=-2,
        x₃_t0=0,
        x₁_tf=0,
        x₂_tf=0,
        x₃_tf=0,
    ),
    :status => (OptimalControl=:valid, OptimalControlScalarized=:valid, JuMP=:valid),
    # initial guess of every modeling (#377), from the parameters
    :init => p -> (state=[0.1, 0.1, 0.1], control=0.1),
)
