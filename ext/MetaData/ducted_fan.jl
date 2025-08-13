ducted_fan_meta = OrderedDict(
    :name => "ducted_fan",
    :N => 250,
    :minimize => true,
    :state_name => ["x₁", "v₁", "x₂", "v₂", "α", "vα"],
    :costate_name => ["∂x₁", "∂v₁", "∂x₂", "∂v₂", "∂α", "∂vα"],
    :control_name => ["u₁", "u₂"],
    :time => ("final_time", "tf", nothing),
)
