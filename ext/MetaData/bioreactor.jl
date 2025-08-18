bioreactor_meta = OrderedDict(
    :name => "bioreactor",
    :N => 500,
    :minimise => false,
    :state_name => ["y", "s", "b"],
    :costate_name => ["∂y", "∂s", "∂b"],
    :control_name => ["u"],
    :variable_name => nothing,
    :final_time => (:fixed, 200), # the final time is 10N where N = 20 by default
)
