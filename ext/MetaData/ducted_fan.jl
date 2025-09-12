ducted_fan_meta = OrderedDict(
    :N => 250,
    :state_name => ["x₁", "v₁", "x₂", "v₂", "α", "vα"],
    :costate_name => ["∂x₁", "∂v₁", "∂x₂", "∂v₂", "∂α", "∂vα"],
    :control_name => ["u₁", "u₂"],
    :variable_name => ["tf"],
    :time_grid_name => Dict(
        :initial_time => "t0", 
        :final_time => "tf", 
        :steps_number => "N"
    ),
    :parameters => (
        t0 = 0,
        r = 0.2,         # [m]
        J = 0.05,        # [kg.m2]
        m = 2.2,         # [kg]
        mg = 4,          # [N]
        μ = 1000,
    ),
)
