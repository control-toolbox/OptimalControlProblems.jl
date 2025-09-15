cart_pendulum_meta = OrderedDict(
    :grid_size => 500,
    :state_name => ["x", "v", "θ", "ω"],
    :costate_name => ["∂x", "∂v", "∂θ", "∂ω"],
    :control_name => ["Fex"],
    :variable_name => ["tf", "ddx"],
    :time_grid_name => Dict(
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
        max_tf = 5,
        max_x = 1,
        max_v = 2,
        tf_l = 0.1,
        x_t0 = 0,
        θ_t0 = 0,
        ω_t0 = 0,
        θ_tf = π,
        ω_tf = 0,
    ),
)
