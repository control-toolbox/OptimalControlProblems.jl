glider_meta = OrderedDict(
    :name => "glider",
    :N => 500,
    :minimize => false,
    :state_name => ["x", "y", "vx", "vy"],
    :costate_name => ["∂x", "∂y", "∂vx", "∂vy"],
    :control_name => ["cL"],
    :time => ("final_time", "tf", nothing),
)
