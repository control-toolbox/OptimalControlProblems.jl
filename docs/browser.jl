using JSON
using DataFrames
using Tables
using OptimalControlProblems

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
        # the modelings where the problem exists, with its status in each (docs/modelings.jl)
        Modelings=[
            (
                name=string(spec.name),
                title=adapter.title,
                status=string(OptimalControlProblems.status(problem_sym)[spec.name]),
            ) for (spec, adapter) in modelings(problem_sym)
        ],
    )
end

"Collects the data of every problem, drafts included (each has a page), as a DataFrame"
function collect_problems_data()
    return DataFrame([collect_problem_data(sym) for sym in problems(; status=:all)])
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
        """```@raw html
---
aside: false
---
```

# [All the problems](@id problems-browser)

Every problem of the collection, drafts included. **Cards** shows the shape of each solution,
drawn from the trajectories the convergence crons publish, with the status of the problem in
each modeling; **Table** sorts and filters them by their features. Open a problem for its
page.

```@raw html
<ProblemsGallery></ProblemsGallery>
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
