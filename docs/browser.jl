using JSON
using DataFrames
using Tables
using OptimalControlProblems
using NLPModels
using CTModels

const BROWSER_FILE = "problems_browser.md"
const BROWSER_PATH = joinpath(@__DIR__, "src", BROWSER_FILE)
const BROWSER_DATA_PATH = joinpath(@__DIR__, "src", "public", "problems.json")

# -------------------------------
# Helpers
# -------------------------------
function sum_namedtuple(nt::NamedTuple)
    return sum(values(nt))
end

function ocp_data_to_json(data_ocp::DataFrame)
    return JSON.json(Tables.columntable(data_ocp))
end

# -------------------------------
# Data collection
# -------------------------------
function collect_problem_data(problem_sym::Symbol)
    # computed by the package from the model (#376), not here
    f = OptimalControlProblems.features(problem_sym)
    c = f.constraints
    return (
        Problem=string(problem_sym),
        State=f.state_dimension,
        Control=f.control_dimension,
        Variable=f.variable_dimension,
        Cost=uppercasefirst(string(f.cost)),
        FinalTime=string(f.final_time),
        DimStateConstraint=c.state_box,
        DimControlConstraint=c.control_box,
        DimVariableConstraint=c.variable_box,
        DimPathConstraint=c.path,
        DimBoundaryConstraint=c.boundary,
        TotalConstraints=sum_namedtuple(c),
    )
end

"Collects OCP data and returns a DataFrame"
function collect_problems_data()
    return DataFrame([collect_problem_data(sym) for sym in problems()])
end

function collect_problems_data_json()
    return ocp_data_to_json(collect_problems_data())
end

# ---------------------------
# Browser page and data output
# ---------------------------
function generate_problems_browser!(path::AbstractString)
    mkpath(dirname(path))
    mkpath(dirname(BROWSER_DATA_PATH))
    write(BROWSER_DATA_PATH, collect_problems_data_json())
    write(
        path,
        """# [Problems browser](@id problems-browser)

Explore the collection by searching, sorting, and filtering the problem characteristics.
Select a row to reveal its constraint breakdown and follow the problem link for details.

```@raw html
<ProblemsBrowser></ProblemsBrowser>
```
""",
    )
    return path
end

"""
with_problems_browser(f::Function)

Generates the browser page and its JSON data for the duration of a documentation build.
"""
function with_problems_browser(f::Function)
    generate_problems_browser!(BROWSER_PATH)
    try
        return f(BROWSER_FILE)
    finally
        isfile(BROWSER_PATH) && rm(BROWSER_PATH)
        isfile(BROWSER_DATA_PATH) && rm(BROWSER_DATA_PATH)
    end
end
