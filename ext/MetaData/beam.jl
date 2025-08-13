beam_meta = OrderedDict(
    :name => "beam",
    :N => 500,
    :minimize => true,
    :state_name => ["x1", "x2"],
    :costate_name => ["∂x1", "∂x2"],
    :control_name => ["u"],
    :time => ("final_time", "tf", 1),
)
