vanderpol_meta = OrderedDict(
    :grid_size => 500,
    :parameters => (t0=0, tf=2, ω=1, ε=1, x₁_t0=1, x₂_t0=0),
    :parameter_info => (
        t0=(unit=nothing, meaning="initial time"),
        tf=(unit=nothing, meaning="final time"),
        ω=(unit=nothing, meaning="frequency of the oscillator"),
        ε=(unit=nothing, meaning="coefficient of the nonlinear damping"),
        x₁_t0=(unit=nothing, meaning="initial value of x₁"),
        x₂_t0=(unit=nothing, meaning="initial value of x₂"),
    ),
    :status => (
        OptimalControl=:valid,
        OptimalControlScalarized=:valid,
        JuMP=:valid,
        ModelingToolkit=:valid,
    ),
    # initial guess of every modeling (#377), from the parameters
    :init => p -> (state=[0.1, 0.1], control=0.1),
)
