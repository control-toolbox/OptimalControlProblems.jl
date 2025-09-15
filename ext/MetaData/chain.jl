chain_meta = OrderedDict(
    :grid_size => 500,
    :state_name => ["x1", "x2", "x3"],
    :costate_name => ["∂x1", "∂x2", "∂x3"],
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
        L = 4,
        a = 1,
        b = 3,
        x2_i = 0,
        x3_i = 0,
    ),
)
