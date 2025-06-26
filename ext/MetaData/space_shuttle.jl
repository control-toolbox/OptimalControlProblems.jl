space_shuttle_meta = Dict(
    :name => "space_shuttle",
    :nh => 503, #:nh => nothing,
    :nvar => nothing,
    :ncon => nothing,
    :minimize => false,
    :state_name => ["scaled_h", "ϕ", "θ", "scaled_v", "γ", "ψ"],
    :costate_name => ["con_dh", "con_dϕ", "con_dθ", "con_dv", "con_dγ", "con_dψ"],
    :control_name => ["α", "β"],
    :time => ("step", "Δt", nothing)
)