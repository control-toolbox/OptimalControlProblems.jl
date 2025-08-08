double_oscillator_meta = Dict(
    :name => "double_oscillator",
    :nh => 100, #:nh => nothing,
    :nvar => nothing,
    :ncon => nothing,
    :minimize => true,
    :state_name => ["x1", "x2", "x3", "x4"],
    :costate_name => ["p1", "p2", "p3", "p4"],
    :control_name => "u",
    :time => ("final_time", "tf", 2π)
)