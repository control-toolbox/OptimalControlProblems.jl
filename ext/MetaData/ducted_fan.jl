ducted_fan_meta = Dict(
    :name => "ducted_fan",
    :nh => 250,
    :nvar => nothing,
    :ncon => nothing,
    :minimize => true,
    :state_name => ["x₁", "v₁", "x₂", "v₂", "α", "vα"],
    :costate_name => ["∂x₁", "∂v₁", "∂x₂", "∂v₂", "∂α", "∂vα"],
    :control_name => ["u₁", "u₂"],
    :time => ("final_time", "tf", nothing)
)