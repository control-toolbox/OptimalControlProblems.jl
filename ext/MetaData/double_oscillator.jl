double_oscillator_meta = OrderedDict(
    :grid_size => 500,
    :state_name => ["x₁", "x₂", "x₃", "x₄"],
    :costate_name => ["∂x₁", "∂x₂", "∂x₃", "∂x₄"],
    :control_name => ["u"],
    :variable_name => nothing,
    :time_grid_name => Dict(
        :initial_time => "t0", 
        :final_time => "tf", 
        :grid_size => "N",
    ),
    :parameters => (
        t0 = 0,
        tf = 2π, 
        m1 = 100,    # [kg]
        m2 = 2,      # [kg]
        c = 0.5,     # [Ns/m]
        k1 = 100,    # [N/m]
        k2 = 3,      # [N/m]
        u_l = -1,
        u_u = 1,
        x₁_t0 = 0,
        x₂_t0 = 0,
    ),
)
