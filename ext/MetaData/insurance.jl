insurance_meta = OrderedDict(
    :name => "insurance",
    :N => 500,
    :minimize => false,
    :state_name => ["I", "m", "x3"],
    :costate_name => ["∂I", "∂m", "∂x3"],
    :control_name => ["h", "R", "H", "U", "dUdR"],
    :variable_name => nothing,
    :final_time => (:fixed, 10),
)
