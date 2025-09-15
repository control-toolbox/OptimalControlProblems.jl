dielectrophoretic_particle_meta = OrderedDict(
    :grid_size => 500,
    :state_components => ["x", "y"],
    :costate_components => ["∂x", "∂y"],
    :control_components => ["u"],
    :variable_components => ["tf"],
    :time_grid_names => Dict(
        :initial_time => "t0", 
        :final_time => "tf", 
        :grid_size => "N",
    ),
    :parameters => (
        t0 = 0,
        x_t0 = 1,
        y_t0 = 0,
        x_tf = 2,
        α = -0.75,
        c = 1,
        u_l = -1,
        u_u = 1,
        tf_l = 0,
    ),
)
