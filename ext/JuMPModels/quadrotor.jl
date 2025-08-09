"""
Quadrotor Problem:
    We want to find the optimal trajectory of a quadrotor to reach a target position.
    The objective is to minimize the final time.
    The problem is formulated as a JuMP model, and can be found [here](https://arxiv.org/pdf/2303.16746)
"""
function OptimalControlProblems.quadrotor(::JuMPBackend; nh::Int=100)

    # parameters
    g = 9.81
    atmin = 0
    atmax = 9.18 * 5
    tiltmax = 1.1 / 2
    dtiltmax = 6 / 2
    p0 = [0, 0, 2.5]
    v0 = [0, 0, 0]
    u0 = [9.81, 0, 0, 0]
    pf = [0.01, 5, 2.5]
    vf = [0, 0, 0]

    # model
    model = JuMP.Model()

    # 
    @variables(
        model,
        begin

            # variable
            0.1 <= tf, (start = 1)

            # state
            p1[0:nh], (start = 0.1)
            p2[0:nh], (start = 0.1)
            p3[0:nh], (start = 0.1)
            v1[0:nh], (start = 0.1)
            v2[0:nh], (start = 0.1)
            v3[0:nh], (start = 0.1)
            -π / 2 <= ϕ[0:nh] <= π / 2, (start = 0.1)
            -π / 2 <= θ[0:nh] <= π / 2, (start = 0.1)

            # control
            atmin <= at[0:nh] <= atmax, (start = 10)
            -dtiltmax <= dϕ[0:nh] <= dtiltmax, (start = 0.1)
            -dtiltmax <= dθ[0:nh] <= dtiltmax, (start = 0.1)
            ψ[0:nh], (start = 0.1)

        end
    )

    # path constraints
    @constraints(
        model,
        begin
            cond_tiltmax[i=0:nh], cos(θ[i]) * cos(ϕ[i]) >= cos(tiltmax)
        end
    )

    # initial and final conditions
    @constraints(
        model,
        begin
            p1_i, p1[0] == p0[1]
            p2_i, p2[0] == p0[2]
            p3_i, p3[0] == p0[3]
            v1_i, v1[0] == v0[1]
            v2_i, v2[0] == v0[2]
            v3_i, v3[0] == v0[3]
            ϕ_i, ϕ[0] == u0[2]
            θ_i, θ[0] == u0[3]
            p1_f, p1[nh] == pf[1]
            p2_f, p2[nh] == pf[2]
            p3_f, p3[nh] == pf[3]
            v1_f, v1[nh] == vf[1]
            v2_f, v2[nh] == vf[2]
            v3_f, v3[nh] == vf[3]
        end
    )

    # dynamics
    @expressions(
        model,
        begin

            #
            step, tf / nh

            # dynamics
            cr[i=0:nh], cos(ϕ[i])
            sr[i=0:nh], sin(ϕ[i])
            cp[i=0:nh], cos(θ[i])
            sp[i=0:nh], sin(θ[i])
            cy[i=0:nh], cos(ψ[i])
            sy[i=0:nh], sin(ψ[i])
            R[i=0:nh],
            [
                (cy[i]*cp[i]) (cy[i] * sp[i] * sr[i]-sy[i] * cr[i]) (cy[i] * sp[i] * cr[i]+sy[i] * sr[i])
                (sy[i]*cp[i]) (sy[i] * sp[i] * sr[i]+cy[i] * cr[i]) (sy[i] * sp[i] * cr[i]-cy[i] * sr[i])
                (-sp[i]) (cp[i]*sr[i]) (cp[i]*cr[i])
            ]
            at_[i=0:nh], R[i] * [0; 0; at[i]]
            g_, [0; 0; -g]
            a[i=0:nh], at_[i] + g_

            # objective
            dc[i=0:nh], 1e-8 * (at[i]^2 + ϕ[i]^2 + θ[i]^2 + ψ[i]^2) + 1e2 * (ψ[i] - u0[3])^2

        end
    )

    @constraints(
        model,
        begin
            ∂p1[i=1:nh], p1[i] == p1[i - 1] + 0.5 * step * (v1[i] + v1[i - 1])
            ∂p2[i=1:nh], p2[i] == p2[i - 1] + 0.5 * step * (v2[i] + v2[i - 1])
            ∂p3[i=1:nh], p3[i] == p3[i - 1] + 0.5 * step * (v3[i] + v3[i - 1])
            ∂v1[i=1:nh], v1[i] == v1[i - 1] + 0.5 * step * (a[i][1] + a[i - 1][1])
            ∂v2[i=1:nh], v2[i] == v2[i - 1] + 0.5 * step * (a[i][2] + a[i - 1][2])
            ∂v3[i=1:nh], v3[i] == v3[i - 1] + 0.5 * step * (a[i][3] + a[i - 1][3])
             ∂ϕ[i=1:nh],  ϕ[i] ==  ϕ[i - 1] + 0.5 * step * (dϕ[i] + dϕ[i - 1])
             ∂θ[i=1:nh],  θ[i] ==  θ[i - 1] + 0.5 * step * (dθ[i] + dθ[i - 1])
        end
    )

    # objective
    @objective(model, Min, tf + 0.5 * step * sum(dc[i] + dc[i-1] for i in 1:nh))

    return model
end
