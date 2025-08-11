robbins_meta = Dict(
    :name => "robbins",
    :nh => 500,
    :nvar => nothing,
    :ncon => nothing,
    :minimize => true,
    :state_name => ["x1", "x2", "x3"],
    :costate_name => ["∂x1", "∂x2", "∂x3"],
    :control_name => ["u"],
    :time => ("final_time", "tf", 10)
)