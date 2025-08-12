moonlander_meta = Dict(
    :name => "moonlander",
    :nh => 500,
    :nvar => nothing,
    :ncon => nothing,
    :minimize => true,
    :state_name => ["p1", "p2", "dp1", "dp2", "θ", "dθ"],
    :costate_name => ["∂p1", "∂p2", "∂dp1", "∂dp2", "∂θ", "∂dθ"],
    :control_name => ["F1", "F2"],
    :time => ("final_time", "tf", nothing),
)
