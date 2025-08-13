dielectrophoretic_particle_meta = OrderedDict(
    :name => "dielectrophoretic_particle",
    :N => 500,
    :minimize => true,
    :state_name => ["x", "y"],
    :costate_name => ["∂x", "∂y"],
    :control_name => "u",
    :time => ("final_time", "tf", nothing),
)
