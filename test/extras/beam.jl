using Pkg
Pkg.activate("test")
using OptimalControl
using NLPModelsIpopt
using Plots

function Beam(a)
    tf = 1

    ocp = @def begin
        t ∈ [0, tf], time
        x ∈ R², state
        u ∈ R, control
        x(0) == [0, 1]
        x(tf) == [0, -1]
        ẋ(t) == [x₂(t), u(t)]
        0 ≤ x₁(t) ≤ a
        ∫(u(t)^2) → min
    end

    return ocp
end

plot(solve(Beam(1/2))) # state constraint inactive
plot(solve(Beam(1/5))) # state constraint active: touch point
plot(solve(Beam(1/8))) # state constraint active: arc
