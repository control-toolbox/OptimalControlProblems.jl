space_shuttle_meta = OrderedDict(
    :name => "space_shuttle",
    :N => 500,
    :minimize => false,
    :state_name => ["scaled_h", "ϕ", "θ", "scaled_v", "γ", "ψ"],
    :costate_name => ["∂h", "∂ϕ", "∂θ", "∂v", "∂γ", "∂ψ"],
    :control_name => ["α", "β"],
    :time => ("final_time", "tf", nothing),
)
