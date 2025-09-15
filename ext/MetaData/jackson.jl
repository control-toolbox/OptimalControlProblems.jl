jackson_meta = OrderedDict(
    :grid_size => 500,
    :state_name => ["a", "b", "x3"],
    :costate_name => ["∂a", "∂b", "∂x3"],
    :control_name => ["u"],
    :variable_name => nothing,
    :time_grid_name => Dict(
        :initial_time => "t0", 
        :final_time => "tf", 
        :steps_number => "N"
    ),
    :parameters => (
        t0 = 0,
        tf = 4,
        k1 = 1,
        k2 = 10,
        k3 = 1,
    ),
)
