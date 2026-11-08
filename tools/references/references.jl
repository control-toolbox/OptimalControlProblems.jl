# Reference objectives and known solutions from the results of the convergence tests (#375)
# and of the robustness tests (#378).
#
# Every convergence or robustness test writes its solves to
# `test/reports/<report>-results.tsv` (#360): one row per problem, modeling and configuration,
# with the grid, the scheme, the solver status and the objective. For a problem and a scheme
# at its default grid, the converged objectives are grouped into solutions: two objectives
# within a relative `OptimalControlProblems.SAME_SOLUTION` are the same solution. The
# reference objective is the best (smallest: every problem of the collection is a
# minimization); the configurations agree when they all reached one solution.
module ReferenceObjectives

using OptimalControlProblems: OptimalControlProblems
using TOML: TOML

const SAME_SOLUTION = OptimalControlProblems.SAME_SOLUTION

"""The tests whose results count: convergence (#360) and robustness (#378)."""
const TESTS = ("test_convergence", "test_robustness")

"""
    read_results(dirs) -> Vector{Dict{String,String}}

Rows of every `*-results.tsv` under `dirs`, read by column name, with the `platform`: the
first directory under the given one (the name of a CI artifact), or `local`.
"""
function read_results(dirs)
    rows = Dict{String,String}[]
    for dir in dirs, (root, _, files) in walkdir(dir), file in files
        endswith(file, "-results.tsv") || continue
        relative = relpath(root, dir)
        platform = relative == "." ? "local" : first(splitpath(relative))
        lines = readlines(joinpath(root, file))
        isempty(lines) && continue
        header = split(lines[1], '\t')
        for line in lines[2:end]
            cells = split(line, '\t')
            length(cells) == length(header) || continue
            row = Dict{String,String}(
                String(h) => String(c) for (h, c) in zip(header, cells)
            )
            row["platform"] = platform
            push!(rows, row)
        end
    end
    return rows
end

"""Relative spread of `values` around the smallest one (absolute when it is zero)."""
function spread(values)
    best = minimum(values)
    return (maximum(values) - best) / (iszero(best) ? 1 : abs(best))
end

"""
    references(rows, defaults; run) -> Dict{Symbol,Dict{String,Any}}

Reference objectives computed from the result `rows`, per problem, as the TOML tables of
`write_references` (one per scheme). `defaults[problem]` is `(grid_size, parameters)`: only
the solves at the default grid count, and the parameters are recorded so that a change of
the problem makes the reference stale. A problem without any converged solve has no entry.
"""
function references(rows, defaults; run::AbstractString)
    groups = Dict{Tuple{Symbol,String},Vector{Dict{String,String}}}()
    for row in rows
        any(t -> startswith(row["test"], t), TESTS) || continue
        row["result"] == "converged" || continue
        problem = Symbol(row["problem"])
        haskey(defaults, problem) || continue
        parse(Int, row["grid_size"]) == defaults[problem].grid_size || continue
        push!(get!(groups, (problem, row["scheme"]), Dict{String,String}[]), row)
    end
    tables = Dict{Symbol,Dict{String,Any}}()
    for ((problem, scheme), group) in groups
        objectives = [parse(Float64, r["objective"]) for r in group]
        s = spread(objectives)
        found = solutions(group)
        results = [
            Dict{String,Any}(
                "modeling" => r["modeling"],
                "configuration" => r["configuration"],
                "platform" => r["platform"],
                "objective" => parse(Float64, r["objective"]),
            ) for
            r in sort(group; by=r -> (r["modeling"], r["configuration"], r["platform"]))
        ]
        get!(tables, problem, Dict{String,Any}())[scheme] = Dict{String,Any}(
            "value" => minimum(objectives),
            "grid_size" => defaults[problem].grid_size,
            "parameters" => defaults[problem].parameters,
            "source" => "computed",
            "agreement" => length(found) == 1,
            "spread" => s,
            "run" => run,
            "solutions" => found,
            "results" => results,
        )
    end
    return tables
end

"""
    solutions(rows) -> Vector{Dict{String,Any}}

The distinct solutions reached by the converged `rows`, best first: consecutive objectives
(sorted) within a relative `SAME_SOLUTION` of the first one of a group are the same solution.
Each with its objective (the best of its group), its number of solves and the
configurations that reached it.
"""
function solutions(rows)
    objective(r) = parse(Float64, r["objective"])
    groups = Vector{Vector{Dict{String,String}}}()
    for r in sort(rows; by=objective)
        if isempty(groups) ||
            OptimalControlProblems._relative_gap(
            objective(r), objective(first(last(groups)))
        ) > SAME_SOLUTION
            push!(groups, [r])
        else
            push!(last(groups), r)
        end
    end
    return [
        Dict{String,Any}(
            "objective" => objective(first(g)),
            "solves" => length(g),
            "configurations" =>
                sort!(unique(["$(r["modeling"]) $(r["configuration"])" for r in g])),
        ) for g in groups
    ]
end

const HEADER = """
# Reference objectives of the problem, one table per scheme, at its default grid.
# Generated by tools/references/update.jl from the convergence and robustness tests; do not
# edit by hand.
"""

"""
    write_references(dir, tables) -> Vector{String}

Write `dir/<problem>.toml` for each problem of `tables`, keeping the schemes of an existing
file that `tables` does not recompute. Return the files written.
"""
function write_references(dir, tables)
    mkpath(dir)
    written = String[]
    for (problem, schemes) in tables
        file = joinpath(dir, "$(problem).toml")
        data = isfile(file) ? TOML.parsefile(file) : Dict{String,Any}()
        merge!(data, schemes)
        open(file, "w") do io
            print(io, HEADER, "\n")
            return TOML.print(io, data; sorted=true)
        end
        push!(written, file)
    end
    return sort!(written)
end

"""Lines of the summary printed by the script: one per problem and scheme."""
function summary(tables)
    lines = String[]
    for problem in sort!(collect(keys(tables))),
        scheme in sort!(collect(keys(tables[problem])))

        t = tables[problem][scheme]
        flag = t["agreement"] ? "" : "  <-- $(length(t["solutions"])) solutions"
        push!(
            lines,
            "$(problem) $(scheme): $(t["value"]) (spread $(round(t["spread"]; sigdigits=3)), " *
            "$(length(t["results"])) solves)$(flag)",
        )
    end
    return lines
end

end # module ReferenceObjectives
