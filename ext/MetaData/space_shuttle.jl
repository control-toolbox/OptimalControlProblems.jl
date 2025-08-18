space_shuttle_meta = OrderedDict(
    :name => "space_shuttle",
    :N => 500,
    :minimise => false,
    :state_name => ["scaled_h", "ϕ", "θ", "scaled_v", "γ", "ψ"],
    :costate_name => ["∂h", "∂ϕ", "∂θ", "∂v", "∂γ", "∂ψ"],
    :control_name => ["α", "β"],
    :variable_name => ["tf"],
    :final_time => (:free, 1), # first component of the variable
)
