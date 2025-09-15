beam_meta = OrderedDict(
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
        tf = 1,
        x₁_l = 0,
        x₁_u = 0.1,
        x₁_t0 = 0,
        x₂_t0 = 1,
        x₁_tf = 0,
        x₂_tf = -1,
    ),
)
