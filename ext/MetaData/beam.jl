beam_meta = Dict(
    :name => "beam",
    :nh => 500,
    :nvar => nothing,
    :ncon => nothing,
    :minimize => true,
    :state_name => ["x1", "x2"],
    :costate_name => ["∂x1", "∂x2"],
    :control_name => ["u"],
    :time => ("final_time", "tf", 1)
)