brachistochrone_meta = OrderedDict(
    :grid_size => 500,
    :parameters => (
        g=9.80665,
        t0=0.0,
        x0=0.0,
        y0=10.0,
        v0=0.0,
        xf=10.0,
        yf=5.0,
        u_min=(-pi),
        u_max=pi,
    ),
    :parameter_info => (
        g=(unit="m/s²", meaning="gravitational acceleration"),
        t0=(unit="s", meaning="initial time"),
        x0=(unit="m", meaning="initial horizontal position px"),
        y0=(unit="m", meaning="initial vertical position py"),
        v0=(unit="m/s", meaning="initial speed"),
        xf=(unit="m", meaning="final horizontal position px"),
        yf=(unit="m", meaning="final vertical position py"),
        u_min=(
            unit="rad",
            meaning="lower bound of the angle u (not read by the models yet, which use −1.57)",
        ),
        u_max=(
            unit="rad",
            meaning="upper bound of the angle u (not read by the models yet, which use 1.57)",
        ),
    ),
    :status => (OptimalControl=:valid, OptimalControlScalarized=:valid, JuMP=:valid),
    # initial guess of every modeling (#377), from the parameters
    :init =>
        p -> begin
            (; t0, x0, y0, v0, xf, yf) = p
            tf = 2.0
            s(t) = (t - t0) / (tf - t0)
            return (
                state=t ->
                    [x0 + s(t) * (xf - x0), y0 + s(t) * (yf - y0), v0 + s(t) * 10.0],
                control=1.57,
                variable=tf,
            )
        end,
)
