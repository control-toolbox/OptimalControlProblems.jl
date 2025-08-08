moonlander_meta = Dict(
    :name => "moonlander",
    :nh => 100,
    :nvar => 610,
    :ncon => 809,
    :minimize => true,
    :state_name => ["p1", "p2", "dp1", "dp2", "θ", "dθ"],
    :costate_name => ["λp1", "λp2", "λdp1", "λdp2", "λθ", "λdθ"],
    :control_name => ["F1", "F2"],
    :time => ("final_time", "tf", nothing)
)