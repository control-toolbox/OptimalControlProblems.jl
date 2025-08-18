robbins_meta = OrderedDict(
    :name => "robbins",
    :N => 500,
    :minimise => true,
    :state_name => ["x1", "x2", "x3"],
    :costate_name => ["∂x1", "∂x2", "∂x3"],
    :control_name => ["u"],
    :variable_name => nothing,
    :final_time => (:fixed, 10),
)
