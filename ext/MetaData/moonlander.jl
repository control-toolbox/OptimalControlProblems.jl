moonlander_meta = Dict(
    :name => "moonlander",
    :nh => 100,
    :nvar => 610,
    :ncon => 809,
    :minimize => true,
    :state_name => ["p1", "p2", "dp1", "dp2", "theta", "dtheta"],
    :costate_name => ["d_p1", "d_p2", "d_dp1", "d_dp2", "d_theta", "d_dtheta"],
    :control_name => ["F1", "F2"],
    :time => ("final_time", "tf", nothing)
)