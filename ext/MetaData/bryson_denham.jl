bryson_denham_meta = OrderedDict(
    :grid_size => 500,
    :parameters => (
        t0=0.0,       # Initial time
        tf=1.0,       # Final time
        x1_t0=0.0,       # Initial position
        x2_t0=1.0,       # Initial velocity
        x1_tf=0.0,       # Final position
        x2_tf=-1.0,      # Final velocity
        x1_max=1 / 9,        # State constraint: x1(t) <= x1_max
    ),
    :status => (OptimalControl=:valid, OptimalControlScalarized=:valid, JuMP=:draft),
    :notes => "JuMP uses ASCII state component names while the OptimalControl model uses Unicode names.",
    # initial guess of every modeling (#377), from the parameters
    :init => p -> (state=[0.0, 0.0], control=0.0),
    :reference_solution =>
        p -> _bryson_denham_solution(
            p.t0, p.tf, (p.x1_t0, p.x2_t0), (p.x1_tf, p.x2_tf), p.x1_max; weight=0.5
        ),
)
