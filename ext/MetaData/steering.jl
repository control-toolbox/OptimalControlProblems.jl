steering_meta = OrderedDict(
    :N => 500,
    :state_name => ["x1", "x2", "x3", "x4"],
    :costate_name => ["∂x1", "∂x2", "∂x3", "∂x4"],
    :control_name => ["u"],
    :variable_name => ["tf"],
    :time_grid_name => Dict(
        :initial_time => "t0", 
        :final_time => "tf", 
        :steps_number => "N"
    ),
    :parameters => (
        t0 = 0,
        a = 100,
        u_min = -π/2,
        u_max = π/2,
        xs = zeros(4),
        xf = [5, 45, 0],
    ),
)
