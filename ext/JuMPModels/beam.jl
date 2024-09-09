"""
The Beam Problem:
    The problem is formulated as a JuMP modeln and can be found [here](https://github.com/control-toolbox/bocop/tree/main/bocop)
"""
function OptimalControlProblems.beam(::JuMPBackend;nh::Int=100)
    # Parameters
    tf = 1

    # Model
    model = JuMP.Model()

    @variables(model, begin
        0.0 <= x1[0:nh] <= 0.1, (start = 0.0)
        x2[0:nh]                , (start = 0.0)
        -10.0 <= u[0:nh] <= 10.0    , (start = 0.0)
    end)

    # Boundary constraints
    @constraints(model, begin
        x1[0] == 0 
        x2[0] == 1
        x1[nh] == 0
        x2[nh] == -1
    end)

    # Dynamics
    @expressions(model, begin
        step, tf/nh
    end)    
    @constraints(model, begin
        con_x1[t=1:nh], x1[t] == x1[t-1] + 0.5 * step[t] * (x2[t] + x2[t-1])
        con_x2[t=1:nh], x2[t] == x2[t-1] + 0.5 * step[t] * (u[t] + u[t-1])
    end)
    

    @objective(model, Min, sum(u[t]^2 for t in 0:nh))

    return model
end