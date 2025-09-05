glider_meta = OrderedDict(
    :name => "glider",
    :N => 500,
    :minimise => false,
    :state_name => ["x", "y", "vx", "vy"],
    :costate_name => ["∂x", "∂y", "∂vx", "∂vy"],
    :control_name => ["cL"],
    :variable_name => ["tf"],
    :final_time => (:free, 1), # first component of the variable
)
