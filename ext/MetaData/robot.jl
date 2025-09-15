robot_meta = OrderedDict(
    :grid_size => 250,
    :state_name => ["ρ", "dρ", "θ", "dθ", "ϕ", "dϕ"],
    :costate_name => ["∂ρ", "∂dρ", "∂θ", "∂dθ", "∂ϕ", "∂dϕ"],
    :control_name => ["uρ", "uθ", "uϕ"],
    :variable_name => ["tf"],
    :time_grid_name => Dict(
        :initial_time => "t0", 
        :final_time => "tf", 
        :steps_number => "N"
    ),
    :parameters => (
        t0 = 0,
        L = 5,
        # Upper bounds on the controls,
        max_uρ = 1,
        max_uθ = 1,
        max_uϕ = 1,
        # Initial positions of the length and the angles for the robot arm,
        ρ0 = 4.5,
        ϕ0 = π/4,
        θf = 2π/3,
    ),
)
