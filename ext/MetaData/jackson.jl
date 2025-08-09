jackson_meta = Dict(
    :name => "jackson",
    :nh => 100,
    :nvar => 404,
    :ncon => 303,
    :minimize => false,
    :state_name => ["a", "b", "x3"],
    :costate_name => ["∂a", "∂b", "∂3"],
    :control_name => ["u"],
    :time => ("final_time", "tf", 4.0)
)