electrical_vehicle_meta = Dict(
    :name => "electrical_vehicle",
    :nh => 100, #:nh => nothing,
    :nvar => nothing,
    :ncon => nothing,
    :minimize => true,
    :state_name => ["x", "v"],
    :costate_name => ["cond_x", "cond_v"],
    :control_name => "u",
    :time => ("final_time", "tf", 1.0)
)