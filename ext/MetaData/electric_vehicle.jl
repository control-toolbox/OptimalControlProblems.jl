electric_vehicle_meta = OrderedDict(
    :name => "electric_vehicle",
    :N => 500,
    :minimize => true,
    :state_name => ["x", "v"],
    :costate_name => ["∂x", "∂v"],
    :control_name => ["u"],
    :variable_name => nothing,
    :final_time => (:fixed, 1),
)
