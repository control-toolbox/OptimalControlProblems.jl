glider_meta = Dict(
    :name => "glider",
    :nh => 100,
    :nvar => 506,
    :ncon => 407,
    :minimize => false,
    :state_name => ["x", "y", "vx", "vy"],
    :costate_name => ["px", "py", "pvx", "pvy"],
    :control_name => ["cL"],
    :time => ("final_time", "tf", nothing)
)
