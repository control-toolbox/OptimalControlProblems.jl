truck_trailer_meta = Dict(
    :name => "truck_trailer",
    :nh => 100, #:nh => nothing,
    :nvar => nothing,
    :ncon => nothing,
    :minimize => true,
    :state_name => ["x2", "y2", "theta2", "theta1", "theta0"],
    :costate_name => ["d_x2", "d_y2", "d_theta2", "d_theta1", "d_theta0"],
    :control_name => ["v0", "delta0"],
    :time => ("final_time", "tf", nothing)
)