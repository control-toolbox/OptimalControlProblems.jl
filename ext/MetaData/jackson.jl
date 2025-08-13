jackson_meta = OrderedDict(
    :name => "jackson",
    :N => 500,
    :minimize => false,
    :state_name => ["a", "b", "x3"],
    :costate_name => ["∂a", "∂b", "∂x3"],
    :control_name => ["u"],
    :time => ("final_time", "tf", 4),
)
