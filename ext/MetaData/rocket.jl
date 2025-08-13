rocket_meta = OrderedDict(
    :name => "rocket",
    :N => 500,
    :minimize => false,
    :state_name => ["h", "v", "m"],
    :costate_name => ["∂h", "∂v", "∂m"],
    :control_name => ["T"],
    :time => ("final_time", "tf", nothing),
)
