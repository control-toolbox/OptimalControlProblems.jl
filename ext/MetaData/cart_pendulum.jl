cart_pendulum_meta = OrderedDict(
    :name => "cart_pendulum",
    :N => 500,
    :minimise => true,
    :state_name => ["x", "v", "θ", "ω"],
    :costate_name => ["∂x", "∂v", "∂θ", "∂ω"],
    :control_name => ["Fex"],
    :variable_name => ["tf", "ddx"],
    :final_time => (:free, 1), # first component of the variable
)
