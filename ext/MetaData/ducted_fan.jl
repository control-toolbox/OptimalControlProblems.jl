ducted_fan_meta = Dict(
    :name => "ducted_fan",
    :nh => 100, # :nh => nothing,
    :nvar => nothing,
    :ncon => nothing,
    :minimize => true,
    :state_name => ["x₁", "v₁", "x₂", "v₂", "α", "vα"],
    :costate_name => ["px₁", "pv₁", "px₂", "pv₂", "pα", "pvα"],
    :control_name => ["u1", "u2"],
    :time => ("final_time", "tf", nothing)
)