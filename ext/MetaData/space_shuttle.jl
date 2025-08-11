space_shuttle_meta = Dict(
    :name => "space_shuttle",
    :nh => 500,
    :nvar => nothing,
    :ncon => nothing,
    :minimize => false,
    :state_name => ["scaled_h", "ϕ", "θ", "scaled_v", "γ", "ψ"],
    :costate_name => ["∂h", "∂ϕ", "∂θ", "∂v", "∂γ", "∂ψ"],
    :control_name => ["α", "β"],
    :time => ("final_time", "tf", nothing)
)