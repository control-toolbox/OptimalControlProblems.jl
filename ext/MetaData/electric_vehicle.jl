electric_vehicle_meta = Dict(
    :name => "electric_vehicle",
    :nh => 100, #:nh => nothing,
    :nvar => nothing,
    :ncon => nothing,
    :minimize => true,
    :state_name => ["x", "v"],
    :costate_name => ["∂x", "∂v"],
    :control_name => "u",
    :time => ("final_time", "tf", 1)
)