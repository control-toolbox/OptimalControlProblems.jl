cart_pendulum_meta = OrderedDict(
    :grid_size => 500,
    :state_components => ["x", "v", "θ", "ω"],
    :costate_components => ["∂x", "∂v", "∂θ", "∂ω"],
    :control_components => ["Fex"],
    :variable_components => ["tf", "ddx"],
    :time_grid_names => Dict(
        :initial_time => "t0", 
        :final_time => "tf", 
        :grid_size => "N",
    ),
    :parameters => (
        t0 = 0,
        g = 9.81,            # gravitation [m/s^2]
        L = 1,               # pendulum length [m]
        m = 1,               # pendulum mass [kg]
        mcart = 0.5,         # cart mass [kg]
        Fex_l = -5,
        Fex_u = 5,
        x_l = -1,
        x_u = 1,
        v_l = -2,
        v_u = 2,
        tf_l = 0.1,
        x_t0 = 0,
        θ_t0 = 0,
        ω_t0 = 0,
        θ_tf = π,
        ω_tf = 0,
    ),
)
