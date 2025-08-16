robot_meta = OrderedDict(
    :name => "robot",
    :N => 250,
    :minimize => true,
    :state_name => ["ρ", "dρ", "θ", "dθ", "ϕ", "dϕ"],
    :costate_name => ["∂ρ", "∂dρ", "∂θ", "∂dθ", "∂ϕ", "∂dϕ"],
    :control_name => ["uρ", "uθ", "uϕ"],
    :variable_name => ["tf"],
    :final_time => (:free, 1), # first component of the variable
)
