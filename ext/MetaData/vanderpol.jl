vanderpol_meta = OrderedDict(
    :grid_size => 500,
    :state_name => ["x₁", "x₂"],
    :costate_name => ["∂x₁", "∂x₂"],
    :control_name => ["u"],
    :variable_name => nothing,
    :time_grid_name => Dict(
        :initial_time => "t0", 
        :final_time => "tf", 
        :grid_size => "N"
    ),
    :parameters => (
        t0 = 0,
        tf = 2,
        ω = 1,
        ε = 1,
        x₁_t0 = 1,
        x₂_t0 = 0,
    ),
)
