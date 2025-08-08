"""
Particle Steering Problem:
    We want to find the optimal trajectory of a particle.
    The objective is to minimize the time taken to achieve a given altitude and terminal velocity.
    The problem is formulated as a JuMP model, and can be found [here](https://github.com/MadNLP/COPSBenchmark.jl/blob/main/src/steering.jl)
"""
function OptimalControlProblems.steering(::JuMPBackend; nh::Int=100)
    a = 100.0
    u_min, u_max = -pi / 2.0, pi / 2.0
    xs = zeros(4)
    xf = [NaN, 5.0, 45.0, 0.0]

    function gen_x0(k, i)
        if i == 1 || i == 4
            return 0.0
        elseif i == 2
            return 5 * k / nh
        elseif i == 3
            return 45.0 * k / nh
        end
    end

    model = JuMP.Model()

    @variable(model, u_min <= u[i=1:(nh + 1)] <= u_max, start = 0.0)   # control
    @variable(model, x1[i=1:(nh + 1)], start = gen_x0(i, 1))           # state x1
    @variable(model, x2[i=1:(nh + 1)], start = gen_x0(i, 2))           # state x2
    @variable(model, x3[i=1:(nh + 1)], start = gen_x0(i, 3))           # state x3
    @variable(model, x4[i=1:(nh + 1)], start = gen_x0(i, 4))           # state x4
    @variable(model, tf, start = 1.0)                                  # final time

    @expression(model, h, tf / nh) # step size
    @objective(model, Min, tf)

    @constraint(model, tf >= 0.0)

    # Dynamics
    @constraints(
        model,
        begin
            con_x1[i=1:nh], x1[i + 1] == x1[i] + 0.5 * h * (x3[i] + x3[i + 1])
            con_x2[i=1:nh], x2[i + 1] == x2[i] + 0.5 * h * (x4[i] + x4[i + 1])
            con_x3[i=1:nh], x3[i + 1] == x3[i] + 0.5 * h * (a * cos(u[i]) + a * cos(u[i + 1]))
            con_x4[i=1:nh], x4[i + 1] == x4[i] + 0.5 * h * (a * sin(u[i]) + a * sin(u[i + 1]))
        end
    )

    # Boundary conditions
    @constraint(model, x1[1] == xs[1])
    @constraint(model, x2[1] == xs[2])
    @constraint(model, x3[1] == xs[3])
    @constraint(model, x4[1] == xs[4])
    @constraint(model, x2[nh + 1] == xf[2])
    @constraint(model, x3[nh + 1] == xf[3])
    @constraint(model, x4[nh + 1] == xf[4])

    return model
end