chain_meta = OrderedDict(
    :grid_size => 500,
    :state_components => ["x₁", "x₂", "x₃"],
    :costate_components => ["∂x₁", "∂x₂", "∂x₃"],
    :control_components => ["u"],
    :variable_components => nothing,
    :time_grid_names => Dict(
        :initial_time => "t0", 
        :final_time => "tf", 
        :grid_size => "N",
    ),
    :parameters => (
        t0 = 0,
        tf = 1, 
        L = 4,
        a = 1,
        b = 3,
        x₂_t0 = 0,
        x₃_t0 = 0,
    ),
)
