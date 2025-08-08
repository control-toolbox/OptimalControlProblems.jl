cart_pendulum_meta = Dict(
    :name => "cart_pendulum",
    :nh => 100,
    :nvar => 507,
    :ncon => 405,
    :minimize => true,
    :state_name => ["x", "v", "θ", "ω"],
    :costate_name => ["px", "pv", "pθ", "pω"],
    :control_name => ["Fex"],
    :time => ("final_time", "tf", nothing)
)
