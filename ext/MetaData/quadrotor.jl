quadrotor_meta = Dict(
    :name => "quadrotor",
    :nh => 60, #:nh => nothing,
    :nvar => nothing,
    :ncon => nothing,
    :minimize => true,
    :state_name => ["p1", "p2", "p3", "v1", "v2", "v3"],
    :costate_name => ["∂p1", "∂p2", "∂p3", "∂v1", "∂v2", "∂v3"],
    :control_name => ["at", "ϕ", "θ", "ψ"],
    :time => ("final_time", "tf", nothing)
)