robbins_meta = OrderedDict(
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
        tf = 10,
        α = 3,
        β = 0,
        γ = 0.5,
    ),
)
