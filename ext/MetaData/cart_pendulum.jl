cart_pendulum_meta = Dict(
    :name => "cart_pendulum",
    :nh => 100,
    :nvar => 507,
    :ncon => 405,
    :minimize => true,
    :state_name => ["x", "dx", "theta", "omega"],
    :costate_name => ["d_x", "d_dx", "d_theta", "d_omega"],
    :control_name => ["Fex"],
    :time => ("final_time", "tf", nothing)
)
