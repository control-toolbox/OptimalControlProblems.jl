ducted_fan_meta = Dict(
    :name => "ducted_fan",
    :nh => 100, # :nh => nothing,
    :nvar => nothing,
    :ncon => nothing,
    :minimize => true,
    :state_name => ["x1", "v1", "x2", "v2", "α", "vα"],
    :costate_name => ["con_x1", "con_v1", "con_x2", "con_v2", "con_α", "con_vα"],
    :control_name => ["u1", "u2"],
    :time => ("final_time", "tf", nothing)
)