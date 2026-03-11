bryson_denham_meta = OrderedDict(
    :grid_size => 500,
    :parameters => (
        t0=0.0,       # Initial time
        tf=1.0,       # Final time
        x1_t0=0.0,       # Initial position
        x2_t0=1.0,       # Initial velocity
        x1_tf=0.0,       # Final position
        x2_tf=-1.0,      # Final velocity
        x1_max=1/9,        # State constraint: x1(t) <= x1_max
    ),
)
