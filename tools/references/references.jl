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
#
# Each solution says how it was obtained (#409): one of the solves that reached it, its
# representative, with its modeling, modeler, solver and every option, its status, iterations,
# time and constraint violation, and the environment of its run (`<report>-environment.toml`:
# Julia, machine, commit, run, versions of the packages). Nothing is written by hand.
module ReferenceObjectives

using OptimalControlProblems: OptimalControlProblems
using TOML: TOML

const SAME_SOLUTION = OptimalControlProblems.SAME_SOLUTION

"""The tests whose results count: convergence (#360) and robustness (#378)."""
const TESTS = ("test_convergence", "test_robustness")

# The columns that say how a solve was configured (#409), written by the tests since #412.
const SETTINGS_COLUMNS = ("modeler", "solver", "options", "violation")

"""
    read_results(dirs) -> Vector{Dict{String,String}}

Rows of every `*-results.tsv` under `dirs`, read by column name, with the `platform`: the
first directory under the given one (the name of a CI artifact), or `local`; and
`environment`, the file `<report>-environment.toml` written by the same run, read with
`environment`. A results file without the columns of #409 is an error: its solves cannot say
how they were obtained.
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
        missing_columns = setdiff(SETTINGS_COLUMNS, header)
        isempty(missing_columns) || error(
            "$(joinpath(root, file)) has no column $(join(missing_columns, ", ")): " *
            "results recorded before #409, solve again",
        )
        environment = joinpath(root, replace(file, "-results.tsv" => "-environment.toml"))
        isfile(environment) || error(
            "$(environment) is missing: its results cannot say how they were obtained"
        )
        for line in lines[2:end]
            cells = split(line, '\t')
            length(cells) == length(header) || continue
            row = Dict{String,String}(
                String(h) => String(c) for (h, c) in zip(header, cells)
            )
            row["platform"] = platform
            row["environment"] = environment
            push!(rows, row)
        end
    end
    return rows
end

"""The environment of the run of `row` (`<report>-environment.toml`), read once per file."""
function environment(row)
    return get!(() -> TOML.parsefile(row["environment"]), ENVIRONMENTS, row["environment"])
end
const ENVIRONMENTS = Dict{String,Dict{String,Any}}()

# Statuses of a solve that stopped at an acceptable point, not at the requested tolerance:
# still a solution (#409), ranked after a strict convergence when one represents a solution.
const ACCEPTABLE = ("acceptable", "ALMOST_LOCALLY_SOLVED", "SOLVED_TO_ACCEPTABLE_LEVEL")

parse_violation(cell) = isempty(cell) ? Inf : (v=parse(Float64, cell); isnan(v) ? Inf : v)

"""
    representative(rows) -> Dict{String,String}

The solve that represents a solution reached by `rows`: a strict convergence before an
acceptable one, then the smallest constraint violation, then the smallest objective; ties
are broken by modeling, configuration and platform, so that the choice does not depend on
the order of the files.
"""
function representative(rows)
    key(r) = (
        r["reason"] in ACCEPTABLE,
        parse_violation(r["violation"]),
        parse(Float64, r["objective"]),
        r["modeling"],
        r["configuration"],
        r["platform"],
    )
    return first(sort(rows; by=key))
end

"""
    reference(row) -> Dict{String,Any}

How the solve `row` was obtained, as the table `reference` of a solution: everything the
results and the environment of its run say, so that it can be solved again.
"""
function reference(row)
    env = environment(row)
    return Dict{String,Any}(
        "objective" => parse(Float64, row["objective"]),
        "modeling" => row["modeling"],
        "configuration" => row["configuration"],
        "modeler" => row["modeler"],
        "solver" => row["solver"],
        "options" => TOML.parse("o = " * row["options"])["o"],
        "status" => row["reason"],
        "iterations" => parse(Int, row["iterations"]),
        "time" => parse(Float64, row["time"]),
        "violation" => parse_violation(row["violation"]),
        "platform" => row["platform"],
        "run" => env["run"],
        "commit" => env["commit"],
        "julia" => env["julia"],
        "os" => env["os"],
        "arch" => env["arch"],
        "cpu" => env["cpu"],
        "versions" => env["packages"],
    )
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
            # the objective of the representative of the best solution: a solve that can be
            # redone (#409), within `SAME_SOLUTION` of the smallest objective
            "value" => first(found)["objective"],
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
Each with the objective of its `representative`, its number of solves, the configurations
that reached it, and how its representative was obtained (`reference`, #409).
"""
function solutions(rows)
    objective(r) = parse(Float64, r["objective"])
    groups = Vector{Vector{Dict{String,String}}}()
    for r in sort(rows; by=objective)
        if isempty(groups) ||
            OptimalControlProblems.References._relative_gap(
            objective(r), objective(first(last(groups)))
        ) > SAME_SOLUTION
            push!(groups, [r])
        else
            push!(last(groups), r)
        end
    end
    return map(groups) do g
        chosen = representative(g)
        return Dict{String,Any}(
            "objective" => objective(chosen),
            "solves" => length(g),
            "configurations" =>
                sort!(unique(["$(r["modeling"]) $(r["configuration"])" for r in g])),
            "reference" => reference(chosen),
        )
    end
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
