# Tools of the comparison of a JuMP solution with an OptimalControl one: tolerances and norms
# (`suite/comparison/test_solution`). Used on the solves of `beam` and on the representative
# problems (#441). The solutions of two solvers are compared on the same NLP only; model
# equivalence is the NLP equivalence of #284 (`common/equivalence.jl`), the components are
# compared by `test_features`, and convergence is tested per modeling.
isdefined(Main, :SolveJuMP) || include(joinpath(@__DIR__, "solve_jump.jl"))
isdefined(Main, :SolveOptimalControl) ||
    include(joinpath(@__DIR__, "solve_optimalcontrol.jl"))

module Comparison

# Tolerances (relative, absolute).
const OBJECTIVE = (rtol=1e-4, atol=1e-6)
const STATE = (rtol=1e-1, atol=1e-6)
# multipliers: same NLP, two solvers; loose, the sign is what matters (#326)
const COSTATE = (rtol=5e-1, atol=1e-3)

"""L² norm of a one-dimensional signal `X` on the time grid `T` (trapezoidal rule)."""
function L2_norm(T, X)
    s = 0.0
    for i in 1:(length(T) - 1)
        s += 0.5 * (X[i]^2 + X[i + 1]^2) * (T[i + 1] - T[i])
    end
    return sqrt(s)
end

within(a, b; rtol, atol) = abs(a - b) < max(0.5 * (abs(a) + abs(b)) * rtol, atol)

function within_L2(t, a, b; rtol, atol)
    d = L2_norm(t, a .- b)
    return d < max(0.5 * (L2_norm(t, a) + L2_norm(t, b)) * rtol, atol)
end

end # module Comparison
