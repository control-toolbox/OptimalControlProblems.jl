ducted_fan_meta = Dict(
    :name => "ducted_fan",
    :nh => 100, # :nh => nothing,
    :nvar => nothing,
    :ncon => nothing,
    :minimize => true,
    :state_name => ["x₁", "v₁", "x₂", "v₂", "α", "vα"],
    :costate_name => ["∂x₁", "∂v₁", "∂x₂", "∂v₂", "∂α", "∂vα"],
    :control_name => ["u1", "u2"],
    :time => ("final_time", "tf", nothing)
)