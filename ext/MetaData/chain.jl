chain_meta = Dict(
    :name => "chain",
    :nh => 100,
    :nvar => 404,
    :ncon => 305,
    :minimize => true,
    :state_name => ["x1", "x2", "x3"],
    :costate_name => ["∂x1", "∂x2", "∂x3"],
    :control_name => ["u"],
    :time => ("final_time", "tf", 1)
)