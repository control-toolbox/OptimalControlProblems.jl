glider_meta = Dict(
    :name => "glider",
    :nh => 500,
    :nvar => nothing,
    :ncon => nothing,
    :minimize => false,
    :state_name => ["x", "y", "vx", "vy"],
    :costate_name => ["∂x", "∂y", "∂vx", "∂vy"],
    :control_name => ["cL"],
    :time => ("final_time", "tf", nothing),
)
