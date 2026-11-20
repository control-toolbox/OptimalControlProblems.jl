beam_meta = OrderedDict(
    :grid_size => 500,
    :parameters => (t0=0, tf=1, x₁_l=0, x₁_u=0.1, x₁_t0=0, x₂_t0=1, x₁_tf=0, x₂_tf=-1),
    :parameter_info => (
        t0=(unit=nothing, meaning="initial time"),
        tf=(unit=nothing, meaning="final time"),
        x₁_l=(unit=nothing, meaning="lower bound of the position x₁"),
        x₁_u=(
            unit=nothing,
            meaning="upper bound of the position x₁ (the state constraint)",
        ),
        x₁_t0=(unit=nothing, meaning="initial position x₁"),
        x₂_t0=(unit=nothing, meaning="initial velocity x₂"),
        x₁_tf=(unit=nothing, meaning="final position x₁"),
        x₂_tf=(unit=nothing, meaning="final velocity x₂"),
    ),
    :status => (
        OptimalControl=:valid,
        OptimalControlScalarized=:valid,
        JuMP=:valid,
        ModelingToolkit=:valid,
    ),
    # initial guess of every modeling (#377), from the parameters
    :init => p -> (state=[0.05, 0.1], control=0.1),
    :reference_solution =>
        p -> _bryson_denham_solution(
            p.t0,
            p.tf,
            (p.x₁_t0, p.x₂_t0),
            (p.x₁_tf, p.x₂_tf),
            p.x₁_u;
            weight=1,
            lower=p.x₁_l,
        ),
)
