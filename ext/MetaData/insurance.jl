insurance_meta = OrderedDict(
    :N => 500,
    :state_name => ["I", "m", "x₃"],
    :costate_name => ["∂I", "∂m", "∂x₃"],
    :control_name => ["h", "R", "H", "U", "dUdR"],
    :variable_name => nothing,
    :time_grid_name => Dict(
        :initial_time => "t0", 
        :final_time => "tf", 
        :steps_number => "N"
    ),
    :parameters => (
        t0 = 0, 
        tf = 10,
        γ = 0.2,
        λ = 0.25,
        h0 = 1.5,
        w = 1,
        s = 10,
        k = 0,
        σ = 0,
        α = 4,
    ),
)
