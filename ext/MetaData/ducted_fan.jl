ducted_fan_meta = Dict(
    :name => "ducted_fan",
    :nh => 100, # :nh => nothing,
    :nvar => nothing,
    :ncon => nothing,
    :minimize => true,
    :state_name => ["x1", "v1", "x2", "v2", "α", "vα"],
    :costate_name => ["px1", "pv1", "px2", "pv2", "pα", "pvα"],
    :control_name => ["u1", "u2"],
    :time => ("final_time", "tf", nothing)
)