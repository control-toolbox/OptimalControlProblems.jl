rocket_meta = OrderedDict(
    :name => "rocket",
    :N => 500,
    :minimise => false,
    :state_name => ["h", "v", "m"],
    :costate_name => ["∂h", "∂v", "∂m"],
    :control_name => ["T"],
    :variable_name => ["tf"],
    :final_time => (:free, 1), # first component of the variable
)
