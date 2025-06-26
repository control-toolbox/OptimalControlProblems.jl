jackson_meta = Dict(
    :name => "jackson",
    :nh => 100,
    :nvar => 404,
    :ncon => 303,
    :minimize => false,
    :state_name => ["a", "b", "x3"],
    :costate_name => ["con_da", "con_db", "con_dx3"],
    :control_name => ["u"],
    :time => ("final_time", "tf", 4.0)
)