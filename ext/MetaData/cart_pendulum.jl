cart_pendulum_meta = Dict(
    :name => "cart_pendulum",
    :nh => 500,
    :nvar => nothing,
    :ncon => nothing,
    :minimize => true,
    :state_name => ["x", "v", "θ", "ω"],
    :costate_name => ["∂x", "∂v", "∂θ", "∂ω"],
    :control_name => ["Fex"],
    :time => ("final_time", "tf", nothing)
)
