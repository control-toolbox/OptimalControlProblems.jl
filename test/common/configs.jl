# The single matrix of test configurations, read by every test (#304): solver options,
# grid of the fast tests, and, per modeling, the configurations to solve with. Extended by
# #283 (modelers, solvers of OptimalControl v2) and #284 (schemes).
# Included at top level, so that `Main.TestConfigs` exists in every environment.
module TestConfigs

# Solver options (values of the former test/runtests.jl).
const TOL = 1e-8
const MU_STRATEGY = "adaptive"
const SB = "yes"
const MAX_ITER = 1000
const MAX_WALL_TIME = 500.0

# Grid of the fast tests (suite/); the slow ones (cron/) use the default grid of each problem.
const SMALL_GRID = 50

# Configurations per modeling. `description` is forwarded to the problem builder
# (OptimalControl: CTDirect description), `solver` selects a `Solve<Solver>` module of
# test/common/. Every model is discretized with its default scheme (trapezoidal); the
# schemes become a dimension of this matrix with #284.
const CONFIGS = (
    JuMP=((solver=:ipopt,),),
    OptimalControl=((description=(), solver=:ipopt),),
    OptimalControlScalarized=((description=(:madnlp, :exa), solver=:madnlp),),
)

"""Name of a configuration, for test sets and reports."""
label(config) = join(("$k=$(v isa Tuple ? join(v, ",") : v)" for (k, v) in pairs(config)), " ")

end # module TestConfigs
