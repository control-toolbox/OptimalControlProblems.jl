robot_meta = OrderedDict(
    :grid_size => 250,
    :parameters => (
        t0=0,
        L=5,
        # Lower and upper bounds on the controls,
        uρ_l=-1,
        uρ_u=1,
        uθ_l=-1,
        uθ_u=1,
        uϕ_l=-1,
        uϕ_u=1,
        # Initial positions of the length and the angles for the robot arm,
        ρ_t0=4.5,
        θ_t0=0,
        ϕ_t0=π / 4,
        dρ_t0=0,
        dθ_t0=0,
        dϕ_t0=0,
        # Final positions
        ρ_tf=4.5,
        θ_tf=2π / 3,
        ϕ_tf=π / 4,
        dρ_tf=0,
        dθ_tf=0,
        dϕ_tf=0,
        #
        ρ_l=0,
        θ_l=(-π),
        θ_u=π,
        ϕ_l=0,
        ϕ_u=π,
        tf_l=0.1,
    ),
    :status => (OptimalControl=:valid, OptimalControlScalarized=:valid, JuMP=:valid),
    # initial guess of every modeling (#377), from the parameters
    :init => p -> begin
        (; t0, ρ_t0, θ_t0, ϕ_t0, ρ_tf, θ_tf, ϕ_tf) = p
        tf = 9.1
        function state(t)
            s = clamp((t - t0) / (tf - t0), 0.0, 1.0)
            ϕ = ϕ_t0 + s * (ϕ_tf - ϕ_t0)
            abs(ϕ) < 1e-6 && (ϕ = 1e-6) # away from zero: the dynamics divide by sin(ϕ)²
            return [ρ_t0 + s * (ρ_tf - ρ_t0), 0.0, θ_t0 + s * (θ_tf - θ_t0), 0.0, ϕ, 0.0]
        end
        return (state=state, control=[0.0, 0.0, 0.0], variable=tf)
    end,
)
