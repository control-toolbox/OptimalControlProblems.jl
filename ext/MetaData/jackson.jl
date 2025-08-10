jackson_meta = Dict(
    :name => "jackson",
    :nh => 500,
    :nvar => nothing,
    :ncon => nothing,
    :minimize => false,
    :state_name => ["a", "b", "x3"],
    :costate_name => ["∂a", "∂b", "∂x3"],
    :control_name => ["u"],
    :time => ("final_time", "tf", 4)
)