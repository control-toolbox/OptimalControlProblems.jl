quadrotor_meta = OrderedDict(
    :name => "quadrotor",
    :N => 50,
    :minimize => true,
    :state_name => ["p₁", "p₂", "p₃", "v₁", "v₂", "v₃", "ϕ", "θ"],
    :costate_name => ["∂p₁", "∂p₂", "∂p₃", "∂v₁", "∂v₂", "∂v₃", "∂ϕ", "∂θ"],
    :control_name => ["at", "dϕ", "dθ", "ψ"],
    :variable_name => ["tf"],
    :final_time => (:free, 1), # first component of the variable
)
