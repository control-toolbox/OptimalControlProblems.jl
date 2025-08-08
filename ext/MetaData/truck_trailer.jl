truck_trailer_meta = Dict(
    :name => "truck_trailer",
    :nh => 100, #:nh => nothing,
    :nvar => nothing,
    :ncon => nothing,
    :minimize => true,
    :state_name => ["x2", "y2", "θ2", "θ1", "θ0"],
    :costate_name => ["px2", "py2", "pθ2", "pθ1", "pθ0"],
    :control_name => ["v0", "δ0"],
    :time => ("final_time", "tf", nothing)
)
