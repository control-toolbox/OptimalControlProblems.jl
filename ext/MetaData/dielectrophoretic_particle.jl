dielectrophoretic_particle_meta = Dict(
    :name => "dielectrophoretic_particle",
    :nh => 500,
    :nvar => nothing,
    :ncon => nothing,
    :minimize => true,
    :state_name => ["x", "y"],
    :costate_name => ["∂x", "∂y"],
    :control_name => "u",
    :time => ("final_time", "tf", nothing),
)
