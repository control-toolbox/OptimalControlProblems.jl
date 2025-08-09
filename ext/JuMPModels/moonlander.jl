"""
The Moonlander Problem:
    We want to find the optimal trajectory for a moonlander to land on the moon.
    The objective is to minimize the time taken to land on the moon.
    The problem is formulated as a JuMP model, and can be found [here](https://arxiv.org/pdf/2303.16746)
"""
function OptimalControlProblems.moonlander(
    ::JuMPBackend; target::Array{Float64}=[5.0, 5.0], nh::Int=500
)
    # parameters
    if size(target) != (2,)
        error("The input target must be of length 2.")
    end
    m = 1
    g = 9.81
    I = 0.1
    D = 1
    max_thrust = 2g

    # define the problem
    model = JuMP.Model()

    # state, control and final time variables
    @variables(
        model,
        begin
            # final time
            0.1 <= tf, (start = 1)

            # state variables
            p1[k=0:nh], (start = 0.1)
            p2[k=0:nh], (start = 0.1)
            dp1[k=0:nh], (start = 0.1)
            dp2[k=0:nh], (start = 0.1)
            θ[k=0:nh], (start = 0.1)
            dθ[k=0:nh], (start = 0.1)

            # control variables
            0 <= F1[k=0:nh] <= max_thrust, (start = 5.0)
            0 <= F2[k=0:nh] <= max_thrust, (start = 5.0)
        end
    )

    # initial and final conditions
    @constraints(
        model,
        begin
            p1[0] == 0
            p2[0] == 0
            dp1[0] == 0
            dp2[0] == 0
            θ[0] == 0
            dθ[0] == 0
            p1[nh] == target[1]
            p2[nh] == target[2]
            dp1[nh] == 0
            dp2[nh] == 0
        end
    )

    # dynamics
    @expressions(
        model,
        begin
            F_r[k=0:nh],
            [
                cos(θ[k]) -sin(θ[k]) p1[k]
                sin(θ[k]) cos(θ[k]) p2[k]
                0 0 1
            ]
        end
    )
    @expressions(
        model,
        begin
            F_tot[k=0:nh], (F_r[k] * [0; F1[k] + F2[k]; 0])[1:2]
        end
    )
    @expressions(
        model,
        begin

            #
            step, tf / nh

            #
            ddp1[k=0:nh], (1 / m) * F_tot[k][1]
            ddp2[k=0:nh], (1 / m) * F_tot[k][2] - g
            ddθ[k=0:nh], (1 / I) * (D / 2) * (F2[k] - F1[k])
            
        end
    )

    @constraints(
        model,
        begin
            ∂p1[k=1:nh],   p1[k] ==  p1[k - 1] + 0.5 * step * ( dp1[k] +  dp1[k - 1])
            ∂p2[k=1:nh],   p2[k] ==  p2[k - 1] + 0.5 * step * ( dp2[k] +  dp2[k - 1])
            ∂dp1[k=1:nh], dp1[k] == dp1[k - 1] + 0.5 * step * (ddp1[k] + ddp1[k - 1])
            ∂dp2[k=1:nh], dp2[k] == dp2[k - 1] + 0.5 * step * (ddp2[k] + ddp2[k - 1])
            ∂θ[k=1:nh],     θ[k] ==   θ[k - 1] + 0.5 * step * (  dθ[k] +   dθ[k - 1])
            ∂dθ[k=1:nh],   dθ[k] ==  dθ[k - 1] + 0.5 * step * ( ddθ[k] +  ddθ[k - 1])
        end
    )

    @objective(model, Min, tf)

    return model
end
