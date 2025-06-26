robot_meta = Dict(
    :name => "robot",
    :nh => 100,
    :nvar => 910,
    :ncon => 612,
    :minimize => true,
    :state_name => ["rho", "rho_dot", "the", "the_dot", "phi", "phi_dot"],
    :costate_name => ["con_rho", "con_rho_dot", "con_the", "con_the_dot", "con_phi", "con_phi_dot"],
    :control_name => ["u_rho", "u_the", "u_phi"],
    :time => ("final_time", "tf", nothing)
)