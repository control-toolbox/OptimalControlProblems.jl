electric_vehicle_meta = OrderedDict(
    :grid_size => 500,
    :state_name => ["x", "v"],
    :costate_name => ["∂x", "∂v"],
    :control_name => ["u"],
    :variable_name => nothing,
    :time_grid_name => Dict(
        :initial_time => "t0", 
        :final_time => "tf", 
        :steps_number => "N"
    ),
    :parameters => (
        t0 = 0,
        tf = 1,
        b1 = 1e0,
        b2 = 1e0,
        h0 = 0.1,
        h1 = 1,
        h2 = 1e-3,
        α0 = 3,
        α1 = 0.4,
        α2 = -1,
        α3 = 0.1,
        x_i = 0,
        v_i = 0,
        x_f = 10,
        v_f = 0,
    ),
)
