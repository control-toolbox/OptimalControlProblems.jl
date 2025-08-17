"""
Robot arm problem:
    We want to find the shape of a robot arm moving between two points.
    The objective is to minimise the time taken to move between the two points.
    The problem is formulated as a JuMP model, and can be found [here](https://github.com/MadNLP/COPSBenchmark.jl/blob/main/src/robot.jl)
"""
function OptimalControlProblems.robot(::JuMPBackend; N::Int=250)

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

    # state, control, variable (final time) and initial guess
    @variables(
        model,
        begin
            0 <= ρ[k = 0:N] <= L, (start = ρ0)
            -π <= θ[k = 0:N] <= π, (start = 2π/3 * (k / N)^2)
            0 <= ϕ[k = 0:N] <= π, (start = ϕ0)

            dρ[k = 0:N], (start = 0)
            dθ[k = 0:N], (start = 4π/3 * (k / N))
            dϕ[k = 0:N], (start = 0)

            -max_uρ <= uρ[0:N] <= max_uρ, (start = 0)
            -max_uθ <= uθ[0:N] <= max_uθ, (start = 0)
            -max_uϕ <= uϕ[0:N] <= max_uϕ, (start = 0)

            tf >= 0.1, (start = 1)
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
            ρ[N] == ρ0
            θ[N] == θf
            ϕ[N] == ϕ0
            dρ[N] == 0
            dθ[N] == 0
            dϕ[N] == 0
        end
    )

    # dynamics
    @expressions(
        model,
        begin

            #
            step, tf / N

            #
            I_θ[i = 0:N], ((L - ρ[i])^3 + ρ[i]^3) * (sin(ϕ[i]))^2
            I_ϕ[i = 0:N], (L - ρ[i])^3 + ρ[i]^3

            #
            ddρ[i = 0:N], uρ[i] / L
            ddθ[i = 0:N], 3 * uθ[i] / I_θ[i]
            ddϕ[i = 0:N], 3 * uϕ[i] / I_ϕ[i]
        end
    )

    @constraints(
        model,
        begin
            ∂ρ[i = 1:N], ρ[i] == ρ[i - 1] + 0.5 * step * (dρ[i] + dρ[i - 1])
            ∂ϕ[i = 1:N], ϕ[i] == ϕ[i - 1] + 0.5 * step * (dϕ[i] + dϕ[i - 1])
            ∂θ[i = 1:N], θ[i] == θ[i - 1] + 0.5 * step * (dθ[i] + dθ[i - 1])
            ∂dρ[i = 1:N], dρ[i] == dρ[i - 1] + 0.5 * step * (ddρ[i] + ddρ[i - 1])
            ∂dθ[i = 1:N], dθ[i] == dθ[i - 1] + 0.5 * step * (ddθ[i] + ddθ[i - 1])
            ∂dϕ[i = 1:N], dϕ[i] == dϕ[i - 1] + 0.5 * step * (ddϕ[i] + ddϕ[i - 1])
        end
    )

    # objective
    @objective(model, Min, tf)

    return model
end
