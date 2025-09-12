dielectrophoretic_particle_meta = OrderedDict(
    :N => 500,
    :state_name => ["x", "y"],
    :costate_name => ["∂x", "∂y"],
    :control_name => ["u"],
    :variable_name => ["tf"],
    :time_grid_name => Dict(
        :initial_time => "t0", 
        :final_time => "tf", 
        :steps_number => "N"
    ),
    :parameters => (
        t0 = 0,
        x0 = 1,
        xf = 2,
        α = -0.75,
        c = 1,
    ),
)
