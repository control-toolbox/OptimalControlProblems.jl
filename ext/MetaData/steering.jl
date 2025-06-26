steering_meta = Dict(
    :name => "steering",
    :nh => 100,
    :nvar => 506,
    :ncon => 408,
    :minimize => true,
    :state_name => ["x1", "x2", "x3", "x4"],
    :costate_name => ["con_x1", "con_x2", "con_x3", "con_x4"],
    :control_name => "u",
    :time => ("final_time", "tf", nothing)
)