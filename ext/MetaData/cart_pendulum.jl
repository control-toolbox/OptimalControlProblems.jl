cart_pendulum_meta = OrderedDict(
    :name => "cart_pendulum",
    :N => 500,
    :minimize => true,
    :state_name => ["x", "v", "θ", "ω"],
    :costate_name => ["∂x", "∂v", "∂θ", "∂ω"],
    :control_name => ["Fex"],
    :time => ("final_time", "tf", nothing),
)
