# Compute the reference objectives from the results of the convergence tests (#375) and write
# them to ext/MetaData/references/<problem>.toml.
#
#     julia --project=. tools/references/update.jl [--run <url>] <dir>...
#
# <dir>: directories holding `*-results.tsv` files, searched recursively: `test/reports`
# after `Pkg.test(test_args=["cron"])`, or the artifacts of the cron runs
# (`gh run download <id> --dir <dir>`, one run per environment). `--run`: where the results
# come from (the cron runs), recorded in the files; `local` by default.

using OptimalControlProblems: OptimalControlProblems

include(joinpath(@__DIR__, "references.jl"))

function main(args)
    run = "local"
    dirs = String[]
    i = 1
    while i <= length(args)
        if args[i] == "--run"
            run = args[i + 1]
            i += 2
        else
            push!(dirs, args[i])
            i += 1
        end
    end
    isempty(dirs) &&
        error("usage: julia --project=. tools/references/update.jl [--run <url>] <dir>...")
    defaults = Dict(
        problem => (
            grid_size=OptimalControlProblems.grid_size_data(problem),
            parameters=repr(OptimalControlProblems.problem_metadata(problem)[:parameters]),
        ) for problem in OptimalControlProblems.problems(; status=:all)
    )
    rows = ReferenceObjectives.read_results(dirs)
    tables = ReferenceObjectives.references(rows, defaults; run=run)
    files = ReferenceObjectives.write_references(
        OptimalControlProblems.References.REFERENCES_PATH, tables
    )
    foreach(println, ReferenceObjectives.summary(tables))
    missing_problems = setdiff(keys(defaults), keys(tables))
    isempty(missing_problems) || println(
        "No converged solve, no reference: ",
        join(sort!(collect(missing_problems)), ", "),
    )
    println("$(length(rows)) results read, $(length(files)) files written.")
    return nothing
end

main(ARGS)
