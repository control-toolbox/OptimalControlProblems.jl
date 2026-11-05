# The single matrix of test configurations, read by every test (#304): solver options,
# grid of the fast tests, and, per modeling, the configurations to solve with. Extended by
# #284 (schemes).
# Included at top level, so that `Main.TestConfigs` exists in every environment.
module TestConfigs

# Solver options (values of the former test/runtests.jl).
const TOL = 1e-8
const MU_STRATEGY = "adaptive"
const SB = "yes"
const MAX_ITER = 1000
# MadNLP on the GPU solves the condensed KKT system (Cholesky, CUDSS), whose conditioning is
# the square of the unreduced one: `TOL` is out of reach for `bioreactor` (#357), 1e-6 is not.
# A GPU solve is first tried at `TOL` (`GPU_FIRST_MAX_ITER` iterations), then at `GPU_TOL`:
# `mountain_car` converges at `TOL` but is declared infeasible at `GPU_TOL`.
const GPU_TOL = 1e-6
const GPU_FIRST_MAX_ITER = 300
const MAX_WALL_TIME = 500.0

# Grid of the fast tests (suite/); the slow ones (cron/) use the default grid of each problem.
const SMALL_GRID = 50

# Problems the tests are restricted to (`problem=a,b` in the test arguments, see
# `TestStatus.select_problems!`); `nothing`: every problem.
const PROBLEMS = Ref{Union{Nothing,Vector{Symbol}}}(nothing)

# Configurations per modeling. JuMP: `solver` selects the optimizer of `SolveJuMP`.
# OptimalControl modelings, explicit mode of OptimalControl v2 (`SolveOptimalControl`):
# `modeler` (`:adnlp`, `:exa`) and `solver` (`:ipopt`, `:madnlp`). Every model is
# discretized with the trapezoidal rule; the schemes become a dimension of this matrix with
# #284.
const CONFIGS = (
    JuMP=((solver=:ipopt,),),
    OptimalControl=((modeler=:adnlp, solver=:ipopt), (modeler=:exa, solver=:madnlp)),
    OptimalControlScalarized=((modeler=:exa, solver=:madnlp),),
)

# GPU configurations of the OptimalControl modelings (environment `GPU`, run on the GPU
# runners only): Exa and MadNLP on the GPU, `SolveOptimalControl` builds them. Apart from
# `CONFIGS`, so that the CPU environments never see them.
const GPU_CONFIGS = (
    OptimalControl=((modeler=:exa_gpu, solver=:madnlp_gpu),),
    OptimalControlScalarized=((modeler=:exa_gpu, solver=:madnlp_gpu),),
)

"""Name of a configuration, for test sets and reports."""
label(config) = join(("$k=$(v isa Tuple ? join(v, ",") : v)" for (k, v) in pairs(config)), " ")

end # module TestConfigs
