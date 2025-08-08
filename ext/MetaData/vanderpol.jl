vanderpol_meta = Dict(
    :name => "vanderpol",
    :nh => 100,
    :nvar => 404,
    :ncon => 303,
    :minimize => true,
    :state_name => ["x1", "x2"],
    :costate_name => ["p1", "p2"],
    :control_name => "u",
    :time => ("final_time", "tf", 2.0)
)