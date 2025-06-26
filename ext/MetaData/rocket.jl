rocket_meta = Dict(
    :name => "rocket",
    :nh => 100,
    :nvar => 405,
    :ncon => 304,
    :minimize => false,
    :state_name => ["h", "v", "m"],
    :costate_name => ["con_dh", "con_dv", "con_dm"],
    :control_name => ["T"],
    :time => ("step", "step", nothing)
)