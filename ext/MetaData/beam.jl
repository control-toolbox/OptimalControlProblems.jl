beam_meta = OrderedDict(
    :grid_size => 500,
    :state_name => ["x1", "x2"],
    :costate_name => ["∂x1", "∂x2"],
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
        x_t0 = [0,  1],
        x_tf = [0, -1],
        x₁_l = 0,
        x₁_u = 0.1,
    ),
)
