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
    :parameter_info => (
        t0=(unit=nothing, meaning="initial time"),
        L=(unit=nothing, meaning="length of the arm, and upper bound of ρ"),
        uρ_l=(unit=nothing, meaning="lower bound of the control uρ"),
        uρ_u=(unit=nothing, meaning="upper bound of the control uρ"),
        uθ_l=(unit=nothing, meaning="lower bound of the control uθ"),
        uθ_u=(unit=nothing, meaning="upper bound of the control uθ"),
        uϕ_l=(unit=nothing, meaning="lower bound of the control uϕ"),
        uϕ_u=(unit=nothing, meaning="upper bound of the control uϕ"),
        ρ_t0=(unit=nothing, meaning="initial length ρ"),
        θ_t0=(unit="rad", meaning="initial horizontal angle θ"),
        ϕ_t0=(unit="rad", meaning="initial vertical angle ϕ"),
        dρ_t0=(unit=nothing, meaning="initial derivative of ρ"),
        dθ_t0=(unit="rad", meaning="initial derivative of θ, per unit of time"),
        dϕ_t0=(unit="rad", meaning="initial derivative of ϕ, per unit of time"),
        ρ_tf=(unit=nothing, meaning="final length ρ"),
        θ_tf=(unit="rad", meaning="final horizontal angle θ"),
        ϕ_tf=(unit="rad", meaning="final vertical angle ϕ"),
        dρ_tf=(unit=nothing, meaning="final derivative of ρ"),
        dθ_tf=(unit="rad", meaning="final derivative of θ, per unit of time"),
        dϕ_tf=(unit="rad", meaning="final derivative of ϕ, per unit of time"),
        ρ_l=(unit=nothing, meaning="lower bound of the length ρ"),
        θ_l=(unit="rad", meaning="lower bound of the angle θ"),
        θ_u=(unit="rad", meaning="upper bound of the angle θ"),
        ϕ_l=(unit="rad", meaning="lower bound of the angle ϕ"),
        ϕ_u=(unit="rad", meaning="upper bound of the angle ϕ"),
        tf_l=(unit=nothing, meaning="lower bound of the final time"),
    ),
    :status => (OptimalControl=:valid, OptimalControlScalarized=:valid, JuMP=:valid),
    # initial guess of every modeling (#377), from the parameters
    :init =>
        p -> begin
            (; t0, ρ_t0, θ_t0, ϕ_t0, ρ_tf, θ_tf, ϕ_tf) = p
            tf = 9.1
            function state(t)
                s = clamp((t - t0) / (tf - t0), 0.0, 1.0)
                ϕ = ϕ_t0 + s * (ϕ_tf - ϕ_t0)
                abs(ϕ) < 1e-6 && (ϕ = 1e-6) # away from zero: the dynamics divide by sin(ϕ)²
                return [
                    ρ_t0 + s * (ρ_tf - ρ_t0), 0.0, θ_t0 + s * (θ_tf - θ_t0), 0.0, ϕ, 0.0
                ]
            end
            return (state=state, control=[0.0, 0.0, 0.0], variable=tf)
        end,
)
