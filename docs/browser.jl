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
    ocp_pb = model(eval(problem_sym)(OptimalControlBackend()))

    cost = if has_mayer_cost(ocp_pb) && has_lagrange_cost(ocp_pb)
        "Bolza"
    elseif has_mayer_cost(ocp_pb)
        "Mayer"
    else
        "Lagrange"
    end

    final_time = has_fixed_final_time(ocp_pb) ? "fixed" : "free"

    dims = (
        x=dim_state_constraints_box(ocp_pb),
        u=dim_control_constraints_box(ocp_pb),
        v=dim_variable_constraints_box(ocp_pb),
        c=dim_path_constraints_nl(ocp_pb),
        b=dim_boundary_constraints_nl(ocp_pb),
    )

    total = sum_namedtuple(dims)

    return (
        Problem=string(problem_sym),
        State=state_dimension(ocp_pb),
        Control=control_dimension(ocp_pb),
        Variable=variable_dimension(ocp_pb),
        Cost=cost,
        FinalTime=final_time,
        DimStateConstraint=dims.x,
        DimControlConstraint=dims.u,
        DimVariableConstraint=dims.v,
        DimPathConstraint=dims.c,
        DimBoundaryConstraint=dims.b,
        TotalConstraints=total,
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
    write(path, """# [Problems browser](@id problems-browser)

Explore the collection by searching, sorting, and filtering the problem characteristics.
Select a row to reveal its constraint breakdown and follow the problem link for details.

```@raw html
<ProblemsBrowser></ProblemsBrowser>
```
""")
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
