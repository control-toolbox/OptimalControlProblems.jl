quadrotor_meta = Dict(
    :name => "quadrotor",
    :nh => 50,
    :nvar => nothing,
    :ncon => nothing,
    :minimize => true,
    :state_name => ["p₁", "p₂", "p₃", "v₁", "v₂", "v₃", "ϕ", "θ"],
    :costate_name => ["∂p₁", "∂p₂", "∂p₃", "∂v₁", "∂v₂", "∂v₃", "∂ϕ", "∂θ"],
    :control_name => ["at", "dϕ", "dθ", "ψ"],
    :time => ("final_time", "tf", nothing)
)