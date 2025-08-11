truck_trailer_meta = Dict(
    :name => "truck_trailer",
    :nh => 200,
    :nvar => nothing,
    :ncon => nothing,
    :minimize => true,
    :state_name => ["x2", "y2", "θ0", "θ1", "θ2", "v0", "δ0"],
    :costate_name => ["∂x2", "∂y2", "∂θ0", "∂θ1", "∂θ2", "∂v0", "∂δ0"],
    :control_name => ["dv0", "dδ0"],
    :time => ("final_time", "tf", nothing)
)
