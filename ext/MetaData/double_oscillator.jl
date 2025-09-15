double_oscillator_meta = OrderedDict(
    :grid_size => 500,
    :state_name => ["x1", "x2", "x3", "x4"],
    :costate_name => ["∂x1", "∂x2", "∂x3", "∂x4"],
    :control_name => ["u"],
    :variable_name => nothing,
    :time_grid_name => Dict(
        :initial_time => "t0", 
        :final_time => "tf", 
        :steps_number => "N"
    ),
    :parameters => (
        t0 = 0,
        tf = 2π, 
        m1 = 100,    # [kg]
        m2 = 2,      # [kg]
        c = 0.5,     # [Ns/m]
        k1 = 100,    # [N/m]
        k2 = 3,      # [N/m]
        u_l = -1,
        u_u = 1,
        x1_i = 0,
        x2_i = 0,
    ),
)
