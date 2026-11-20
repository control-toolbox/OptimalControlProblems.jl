# The skeleton of a new problem, or of a new modeling of a problem (#449, T3).
#
#   julia --project tools/new/problem.jl new <problem> <modelings...>   # a new problem
#   julia --project tools/new/problem.jl add <problem> <modelings...>   # a new modeling of it
#
# The modelings are those of `MODELING_SPECS`: OptimalControl, OptimalControlScalarized, JuMP,
# ModelingToolkit. The files are those of `ext/README.md`, from the templates of `templates/`,
# each modeling `:draft`; their `TODO`s mark what is left to write. Then lint the problem,
# `julia --project tools/lint/problems.jl <problem>`, fill what it reports, and again.
using OptimalControlProblems: OptimalControlProblems

include(joinpath(@__DIR__, "NewProblem.jl"))

const USAGE = "usage: julia --project tools/new/problem.jl new|add <problem> <modelings...>"

length(ARGS) >= 2 && ARGS[1] in ("new", "add") || (println(stderr, USAGE); exit(2))
command, problem, modelings = ARGS[1], Symbol(ARGS[2]), Symbol.(ARGS[3:end])
paths = try
    if command == "new"
        NewProblem.create(problem, modelings)
    else
        NewProblem.add(problem, modelings)
    end
catch e
    e isa ArgumentError || rethrow()
    println(stderr, e.msg, "\n", USAGE)
    exit(2)
end
println("Written:")
foreach(p -> println("- ", relpath(p, pkgdir(OptimalControlProblems))), paths)
println(
    "\nNext: fill the `TODO`s, then `julia --project tools/lint/problems.jl $(problem)` ",
    "until no failure, then the tests (ext/README.md).",
)
