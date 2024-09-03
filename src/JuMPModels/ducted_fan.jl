"""
The Ducted Fan Problem:
    Implement the optimal control of a planar ducted fan.
    Instance taken from [GP2009].
    The problem is formulated as a JuMP model.
Ref: Graichen, K., & Petit, N. (2009). Incorporating a class of constraints into the dynamics of optimal control problems. Optimal Control Applications and Methods, 30(6), 537-561.
"""
function ducted_fan(;nh::Int=100)
    r = 0.2         # [m]
    J = 0.05        # [kg.m2]
    m = 2.2         # [kg]
    mg = 4.0        # [N]
    μ = 1000.0

    model = Model()

    @variable(model, x1[0:nh])
    @variable(model, v1[0:nh])
    @variable(model, x2[0:nh])
    @variable(model, v2[0:nh])
    @variable(model, -deg2rad(30.0) <= α[0:nh] <= deg2rad(30.0)) # radian
    @variable(model, vα[0:nh])
    @variable(model, -5.0 <= u1[0:nh] <= 5.0) # [nh]
    @variable(model, 0.0 <= u2[0:nh] <= 17.0) # [nh]
    @variable(model, 0 <= tf, start=1.0)

    @objective(model, Min, tf / nh * sum(2*u1[t]^2 + u2[t]^2 for t in 0:nh) + μ * tf)

    # Dynamics
    @expressions(model, begin
        step,       tf / nh
        dx1[t=0:nh], v1[t]
        dv1[t=0:nh], (u1[t] * cos(α[t]) - u2[t] * sin(α[t])) / m
        dx2[t=0:nh], v2[t]
        dv2[t=0:nh], (- mg + u1[t] * sin(α[t]) + u2[t] * cos(α[t])) / m
        dα[t=0:nh], vα[t]
        dvα[t=0:nh], r * u1[t] / J
    end)
    # Collocation
    @constraints(model, begin
        con_x1[t=1:nh], x1[t] == x1[t-1] + 0.5 * step * (dx1[t] + dx1[t-1])
        con_v1[t=1:nh], v1[t] == v1[t-1] + 0.5 * step * (dv1[t] + dv1[t-1])
        con_x2[t=1:nh], x2[t] == x2[t-1] + 0.5 * step * (dx2[t] + dx2[t-1])
        con_v2[t=1:nh], v2[t] == v2[t-1] + 0.5 * step * (dv2[t] + dv2[t-1])
        con_α[t=1:nh], α[t] == α[t-1] + 0.5 * step * (dα[t] + dα[t-1])
        con_vα[t=1:nh], vα[t] == vα[t-1] + 0.5 * step * (dvα[t] + dvα[t-1])
    end)
    # Boundary constraints
    @constraints(model, begin
        x1[0] == 0.0
        x2[0] == 0.0
        α[0] == 0.0
        v1[0] == 0.0
        v2[0] == 0.0
        vα[0] == 0.0
        x1[nh] == 1.0
        x2[nh] == 0.0
        α[nh] == 0.0
        v1[nh] == 0.0
        v2[nh] == 0.0
        vα[nh] == 0.0
    end)

    return model
end

