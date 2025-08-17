steering_meta = OrderedDict(
    :name => "steering",
    :N => 500,
    :minimise => true,
    :state_name => ["x1", "x2", "x3", "x4"],
    :costate_name => ["∂x1", "∂x2", "∂x3", "∂x4"],
    :control_name => ["u"],
    :variable_name => ["tf"],
    :final_time => (:free, 1), # first component of the variable
)
