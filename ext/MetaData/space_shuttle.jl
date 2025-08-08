space_shuttle_meta = Dict(
    :name => "space_shuttle",
    :nh => 503, #:nh => nothing,
    :nvar => nothing,
    :ncon => nothing,
    :minimize => false,
    :state_name => ["scaled_h", "ϕ", "θ", "scaled_v", "γ", "ψ"],
    :costate_name => ["ph", "pϕ", "pθ", "pv", "pγ", "pψ"],
    :control_name => ["α", "β"],
    :time => ("step", "Δt", nothing)
)