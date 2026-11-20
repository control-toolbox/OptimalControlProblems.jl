# The linter of the problems (#449): what a problem must have, checked for every problem.
#
#   julia --project tools/lint/problems.jl                 # every problem
#   julia --project tools/lint/problems.jl beam chain      # some problems
#   julia --project tools/lint/problems.jl --verbose beam  # every check, not only failures
#
# Prints a Markdown report: the checks of the linter (✅/❌), what other tools check (☑️) and
# what is left to check by hand (ℹ️). Exits with 1 when a check fails. The checks are in
# `ProblemLint.jl`; `test/suite/core/test_lint.jl` runs them on the collection and on
# examples.
using OptimalControlProblems: OptimalControlProblems

include(joinpath(@__DIR__, "ProblemLint.jl"))

verbose = "--verbose" in ARGS
names = Symbol.(filter(!=("--verbose"), ARGS))
problems = isempty(names) ? OptimalControlProblems.problems(; status=:all) : names
unknown = setdiff(problems, OptimalControlProblems.problems(; status=:all))
isempty(unknown) || error("unknown problems: $(join(unknown, ", "))")

results = ProblemLint.lint(problems)
print(ProblemLint.report(results; verbose))
exit(any(r -> r.status === :fail, results) ? 1 : 0)
