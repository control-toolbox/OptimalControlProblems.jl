# Solver options shared by the test environments (values of the former test/runtests.jl).
# Included at top level, so that `Main.TestSolverOptions` exists in every environment.
module TestSolverOptions
const TOL = 1e-8
const MU_STRATEGY = "adaptive"
const SB = "yes"
const MAX_ITER = 1000
const MAX_WALL_TIME = 500.0
end
