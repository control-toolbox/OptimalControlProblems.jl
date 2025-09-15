cart_pendulum_meta = OrderedDict(
    :grid_size => 500,
    :state_name => ["x", "v", "θ", "ω"],
    :costate_name => ["∂x", "∂v", "∂θ", "∂ω"],
    :control_name => ["Fex"],
    :variable_name => ["tf", "ddx"],
    :time_grid_name => Dict(
        :initial_time => "t0", 
        :final_time => "tf", 
        :steps_number => "N",
    ),
    :parameters => (
        t0 = 0,
        g = 9.81,            # gravitation [m/s^2]
        L = 1,               # pendulum length [m]
        m = 1,               # pendulum mass [kg]
        mcart = 0.5,         # cart mass [kg]
        max_f = 5,
        max_x = 1,
        max_v = 2,
        tf_l = 0.1,
        x_i = 0,
        θ_i = 0,
        ω_i = 0,
        θ_f = π,
        ω_f = 0,
    ),
)
