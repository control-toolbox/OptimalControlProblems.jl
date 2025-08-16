dielectrophoretic_particle_meta = OrderedDict(
    :name => "dielectrophoretic_particle",
    :N => 500,
    :minimize => true,
    :state_name => ["x", "y"],
    :costate_name => ["∂x", "∂y"],
    :control_name => ["u"],
    :variable_name => ["tf"],
    :final_time => (:free, 1), # first component of the variable
)
