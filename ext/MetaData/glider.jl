glider_meta = Dict(
    :name => "glider",
    :nh => 100,
    :nvar => 506,
    :ncon => 407,
    :minimize => false,
    :state_name => ["x", "y", "vx", "vy"],
    :costate_name => ["x_eqn", "y_eqn", "vx_eqn", "vy_eqn"],
    :control_name => ["cL"],
    :time => ("final_time", "tf", nothing)
)
