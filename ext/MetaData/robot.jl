robot_meta = Dict(
    :name => "robot",
    :nh => 250,
    :nvar => nothing,
    :ncon => nothing,
    :minimize => true,
    :state_name => ["ρ", "dρ", "θ", "dθ", "ϕ", "dϕ"],
    :costate_name => ["∂ρ", "∂dρ", "∂θ", "∂dθ", "∂ϕ", "∂dϕ"],
    :control_name => ["uρ", "uθ", "uϕ"],
    :time => ("final_time", "tf", nothing)
)

