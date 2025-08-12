bioreactor_meta = Dict(
    :name => "bioreactor",
    :nh => 500,
    :nvar => nothing,
    :ncon => nothing,
    :minimize => false,
    :state_name => ["y", "s", "b"],
    :costate_name => ["∂y", "∂s", "∂b"],
    :control_name => ["u"],
    :time => ("final_time", "T", 200), # T = 10 * N where N = 20 by default
)
