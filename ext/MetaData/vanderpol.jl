vanderpol_meta = OrderedDict(
    :N => 500,
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
        tf = 2,
        ω = 1,
        ε = 1,
    ),
)
