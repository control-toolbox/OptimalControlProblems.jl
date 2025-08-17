double_oscillator_meta = OrderedDict(
    :name => "double_oscillator",
    :N => 500,
    :minimise => true,
    :state_name => ["x1", "x2", "x3", "x4"],
    :costate_name => ["∂x1", "∂x2", "∂x3", "∂x4"],
    :control_name => ["u"],
    :variable_name => nothing,
    :final_time => (:fixed, 2π),
)
