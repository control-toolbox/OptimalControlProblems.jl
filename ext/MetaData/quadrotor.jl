quadrotor_meta = Dict(
    :name => "quadrotor",
    :nh => 100,
    :nvar => nothing,
    :ncon => nothing,
    :minimize => true,
    :state_name => ["p1", "p2", "p3", "v1", "v2", "v3", "ϕ", "θ"],
    :costate_name => ["∂p1", "∂p2", "∂p3", "∂v1", "∂v2", "∂v3", "∂ϕ", "∂θ"],
    :control_name => ["at", "dϕ", "dθ", "ψ"],
    :time => ("final_time", "tf", nothing)
)