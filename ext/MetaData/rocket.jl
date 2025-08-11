rocket_meta = Dict(
    :name => "rocket",
    :nh => 500,
    :nvar => nothing,
    :ncon => nothing,
    :minimize => false,
    :state_name => ["h", "v", "m"],
    :costate_name => ["∂h", "∂v", "∂m"],
    :control_name => ["T"],
    :time => ("final_time", "tf", nothing)
)