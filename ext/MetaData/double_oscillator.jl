double_oscillator_meta = OrderedDict(
    :N => 500,
    :state_name => ["x1", "x2", "x3", "x4"],
    :costate_name => ["∂x1", "∂x2", "∂x3", "∂x4"],
    :control_name => ["u"],
    :variable_name => nothing,
    :time_grid_name => Dict(
        :initial_time => "t0", 
        :final_time => "T", 
        :steps_number => "N"
    ),
    :parameters => (
        t0 = 0,
        T = 2π, 
        m1 = 100,    # [kg]
        m2 = 2,      # [kg]
        c = 0.5,     # [Ns/m]
        k1 = 100,    # [N/m]
        k2 = 3,      # [N/m]
    ),
)
