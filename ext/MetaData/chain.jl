chain_meta = OrderedDict(
    :grid_size => 500,
    :parameters => (t0=0, tf=1, L=4, a=1, b=3, x₂_t0=0, x₃_t0=0),
    :status => (OptimalControl=:valid, OptimalControlScalarized=:valid, JuMP=:valid),
    # initial guess of every modeling (#377), from the parameters
    :init => p -> begin
        (; t0, tf, a, b) = p
        tmin = b > a ? 1 / 4 : 3 / 4
        s(t) = (t - t0) / (tf - t0)
        u(t) = 4 * abs(b - a) * (s(t) - tmin)
        x₁(t) = 4 * abs(b - a) * s(t) * (0.5 * s(t) - tmin) + a
        return (state=t -> [x₁(t), x₁(t) * u(t), u(t)], control=u)
    end,
)
