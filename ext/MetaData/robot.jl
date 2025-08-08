robot_meta = Dict(
    :name => "robot",
    :nh => 100,
    :nvar => 910,
    :ncon => 612,
    :minimize => true,
    :state_name => ["ρ", "dρ", "θ", "dθ", "ϕ", "dϕ"],
    :costate_name => ["pρ", "pdρ", "pθ", "pdθ", "pϕ", "pdϕ"],
    :control_name => ["uρ", "uθ", "uϕ"],
    :time => ("final_time", "tf", nothing)
)

