moonlander_meta = OrderedDict(
    :name => "moonlander",
    :N => 500,
    :minimize => true,
    :state_name => ["p1", "p2", "dp1", "dp2", "θ", "dθ"],
    :costate_name => ["∂p1", "∂p2", "∂dp1", "∂dp2", "∂θ", "∂dθ"],
    :control_name => ["F1", "F2"],
    :time => ("final_time", "tf", nothing),
)
