vanderpol_meta = OrderedDict(
    :grid_size => 500,
    :state_components => ["x₁", "x₂"],
    :costate_components => ["∂x₁", "∂x₂"],
    :control_components => ["u"],
    :variable_components => nothing,
    :time_grid_names => Dict(
        :initial_time => "t0", 
        :final_time => "tf", 
        :grid_size => "N",
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
