rocket_meta = OrderedDict(
    :grid_size => 500,
    :state_components => ["h", "v", "m"],
    :costate_components => ["∂h", "∂v", "∂m"],
    :control_components => ["T"],
    :variable_components => ["tf"],
    :time_grid_names => Dict(:initial_time => "t0", :final_time => "tf", :grid_size => "N"),
    :parameters =>
        (t0=0, h_t0=1, v_t0=0, m_t0=1, g0=1, Tc=3.5, hc=500, vc=620, mc=0.6, T_l=0, tf_l=0),
)
