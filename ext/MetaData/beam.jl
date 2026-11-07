beam_meta = OrderedDict(
    :grid_size => 500,
    :parameters => (t0=0, tf=1, x₁_l=0, x₁_u=0.1, x₁_t0=0, x₂_t0=1, x₁_tf=0, x₂_tf=-1),
    :status => (OptimalControl=:valid, OptimalControlScalarized=:valid, JuMP=:valid),
    # initial guess of every modeling (#377), from the parameters
    :init => p -> (state=[0.05, 0.1], control=0.1),
)
