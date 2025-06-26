bioreactor_meta = Dict(
    :name => "bioreactor",
    :nh => 100,
    :nvar => 505,
    :ncon => 404,
    :minimize => false,
    :state_name => ["y", "s", "b"],
    :costate_name => ["con_y", "con_s", "con_b"],
    :control_name => ["u"],
    :time => ("final_time", "T", 300.0) # T = 10 * N where N = 30 by default
)