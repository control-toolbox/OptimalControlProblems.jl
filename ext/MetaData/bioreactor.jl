bioreactor_meta = OrderedDict(
    :grid_size => 600,
    :state_name => ["y", "s", "b"],
    :costate_name => ["∂y", "∂s", "∂b"],
    :control_name => ["u"],
    :variable_name => nothing,
    :time_grid_name => Dict(
        :initial_time => "t0", 
        :final_time => "tf", 
        :steps_number => "N"
    ),
    :parameters => (
        t0 = 0,
        tf = 200, # the final time is 10N where N = 20 by default
        β = 1,
        c = 2,
        γ = 1,
        halfperiod = 5,
        Ks = 0.05,
        μ2m = 0.1,
        μbar = 1,
        r = 0.005,
        x_l = [0, 0, 0.001],
        u_l = 0,
        u_u = 1,
        x0_l = [0.05, 0.5, 0.5],
        x0_u = [0.25, 5, 3],
    ), 
)
