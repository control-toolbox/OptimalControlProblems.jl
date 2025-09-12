moonlander_meta = OrderedDict(
    :N => 500,
    :state_name => ["p1", "p2", "dp1", "dp2", "θ", "dθ"],
    :costate_name => ["∂p1", "∂p2", "∂dp1", "∂dp2", "∂θ", "∂dθ"],
    :control_name => ["F1", "F2"],
    :variable_name => ["tf"],
    :time_grid_name => Dict(
        :initial_time => "t0", 
        :final_time => "tf", 
        :steps_number => "N"
    ),
    :parameters => (
        t0 = 0,
        target=[5.0, 5.0],
        m = 1,
        g = 9.81,
        I = 0.1,
        D = 1,
        max_thrust = 2*9.81,
    ),
)
