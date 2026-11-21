# TODO: the metadata of the problem, in place of those of the example (see ext/README.md).
{{problem}}_meta = OrderedDict(
    :grid_size => 100,
    # every bound of the problem is a parameter, never a number written in a model
    :parameters => (t0=0, tf=1, x_t0=1, x_u=2),
    :parameter_info => (
        t0=(unit=nothing, meaning="initial time"),
        tf=(unit=nothing, meaning="final time"),
        x_t0=(unit=nothing, meaning="initial value of x"),
        x_u=(unit=nothing, meaning="upper bound of x"),
    ),
    {{status}}
    # initial guess of every modeling (#377), from the parameters
    :init => p -> (state=p.x_t0, control=0.0),
)
