insurance_meta = OrderedDict(
    :name => "insurance",
    :N => 500,
    :minimize => false,
    :state_name => ["I", "m", "x3"],
    :costate_name => ["∂I", "∂m", "∂x3"],
    :control_name => ["h", "R", "H", "U", "dUdR"],
    :time => ("final_time", "tf", 10),
)
