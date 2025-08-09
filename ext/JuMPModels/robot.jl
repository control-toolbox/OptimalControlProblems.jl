"""
Robot arm problem:
    We want to find the shape of a robot arm moving between two points.
    The objective is to minimize the time taken to move between the two points.
    The problem is formulated as a JuMP model, and can be found [here](https://github.com/MadNLP/COPSBenchmark.jl/blob/main/src/robot.jl)
"""
function OptimalControlProblems.robot(::JuMPBackend; nh::Int=250)

    # parameters

    # total length of arm
    L = 5

    # Upper bounds on the controls
    max_uρ = 1
    max_uθ = 1
    max_uϕ = 1

    # Initial positions of the length and the angles for the robot arm
    ρ0 = 4.5
    ϕ0 = π/4
    θf = 2π/3

    # model
    model = JuMP.Model()

    # final time: starting value
    tf_start = 1

    # state, control, variable (final time) and initial guess
    @variables(
        model,
        begin
             0 <= ρ[k=0:nh] <= L,               (start = ρ0)
            -π <= θ[k=0:nh] <= π,               (start = 2π/3 * (k * tf_start / nh)^2)
             0 <= ϕ[k=0:nh] <= π,               (start = ϕ0)
            dρ[k=0:nh],                         (start = 0)
            dθ[k=0:nh],                         (start = 4π/3 * (k * tf_start / nh))
            dϕ[k=0:nh],                         (start = 0)
            -max_uρ <= uρ[0:nh] <= max_uρ,   (start = 0)
            -max_uθ <= uθ[0:nh] <= max_uθ,   (start = 0)
            -max_uϕ <= uϕ[0:nh] <= max_uϕ,   (start = 0)
            tf >= 0.1,                          (start = tf_start)
        end
    )

    # Boundary condition
    @constraints(
        model,
        begin

            # initial
            ρ[0] == ρ0
            ϕ[0] == ϕ0
            θ[0] == 0
            dρ[0] == 0
            dθ[0] == 0
            dϕ[0] == 0

            # final
            ρ[nh] == ρ0
            θ[nh] == θf
            ϕ[nh] == ϕ0
            dρ[nh] == 0
            dθ[nh] == 0
            dϕ[nh] == 0

        end
    )

    # dynamics
    @expressions(
        model,
        begin

            #
            step, tf / nh

            #
            I_θ[i=0:nh], ((L - ρ[i])^3 + ρ[i]^3) * (sin(ϕ[i]))^2
            I_ϕ[i=0:nh], (L - ρ[i])^3 + ρ[i]^3

            #
            ddρ[i=0:nh], uρ[i] / L
            ddθ[i=0:nh], 3 * uθ[i] / I_θ[i]
            ddϕ[i=0:nh], 3 * uϕ[i] / I_ϕ[i]

        end
    )

    @constraints(
        model,
        begin
            ∂ρ[i=1:nh],   ρ[i] ==  ρ[i - 1] + 0.5 * step * ( dρ[i] +  dρ[i - 1])
            ∂ϕ[i=1:nh],   ϕ[i] ==  ϕ[i - 1] + 0.5 * step * ( dϕ[i] +  dϕ[i - 1])
            ∂θ[i=1:nh],   θ[i] ==  θ[i - 1] + 0.5 * step * ( dθ[i] +  dθ[i - 1])
            ∂dρ[i=1:nh], dρ[i] == dρ[i - 1] + 0.5 * step * (ddρ[i] + ddρ[i - 1])
            ∂dθ[i=1:nh], dθ[i] == dθ[i - 1] + 0.5 * step * (ddθ[i] + ddθ[i - 1])
            ∂dϕ[i=1:nh], dϕ[i] == dϕ[i - 1] + 0.5 * step * (ddϕ[i] + ddϕ[i - 1])
        end
    )

    # objective
    @objective(model, Min, tf)

    return model
end
