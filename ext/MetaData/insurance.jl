insurance_meta = Dict(
    :name => "insurance",
    :nh => 100, #:nh => nothing,
    :nvar => nothing,
    :ncon => nothing,
    :minimize => false,
    :state_name => ["I", "m", "x3"],
    :costate_name => ["∂I", "∂m", "∂x3"],
    :control_name => ["h", "R", "H", "U", "dUdR"],
    :time => ("final_time", "tf", 10)
)