truck_trailer_meta = OrderedDict(
    :name => "truck_trailer",
    :N => 200,
    :minimize => true,
    :state_name => ["x2", "y2", "θ0", "θ1", "θ2", "v0", "δ0"],
    :costate_name => ["∂x2", "∂y2", "∂θ0", "∂θ1", "∂θ2", "∂v0", "∂δ0"],
    :control_name => ["dv0", "dδ0"],
    :time => ("final_time", "tf", nothing),
)
