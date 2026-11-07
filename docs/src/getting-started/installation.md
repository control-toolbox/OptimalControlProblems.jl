# [Installation](@id installation)

OptimalControlProblems.jl is a registered Julia package. In the Julia REPL:

```julia
using Pkg
Pkg.add("OptimalControlProblems")
```

The package alone lists the problems and reads their metadata: names, parameters, statuses,
initial guess and reference objective. To build a problem in a modeling, you also need the
package of that modeling: each modeling is an **extension**, loaded when its package is.

## What to install, for what

```@eval
using Markdown
rows = map(Main.modelings()) do (spec, adapter)
    "| $(adapter.title) | `$(spec.weakdep)` | $(join(("`$p`" for p in adapter.packages), ", ")) |"
end
Markdown.parse("""
| To use the modeling | the extension needs | and to solve, load |
| :--- | :--- | :--- |
$(join(rows, "\n"))
""")
```

For instance, to build the problems with OptimalControl and solve them with Ipopt:

```julia
Pkg.add(["OptimalControl", "NLPModelsIpopt"])
```

and with JuMP:

```julia
Pkg.add(["JuMP", "Ipopt"])
```

## Other packages

| To | load |
| :--- | :--- |
| solve the OptimalControl modelings on a GPU | `MadNLPGPU`, `CUDA` and `CUDSS` (see [Solve with OptimalControl](@ref solve-optimalcontrol)) |
| plot a solution | `Plots` (see [Plot a solution](@ref plot-solution)) |

When a problem is asked for in a modeling whose package is not loaded, the error says which
package to load:

```@eval
Main.CTBase.DevTools.repl_without_extensions(:OptimalControlProblems, "beam(JuMPBackend())")
```
