# OptimalControlProblems.jl

The OptimalControlProblems.jl package is part of the [control-toolbox ecosystem](https://github.com/control-toolbox) and is independent of [OptimalControl.jl](https://control-toolbox.org/OptimalControl.jl). The control-toolbox ecosystem gathers Julia packages for mathematical control and applications. It aims to provide tools to model and solve optimal control problems with ordinary differential equations by direct and indirect methods. If you want to define an optimal control problem and solve it, please check the [documentation](https://control-toolbox.org/OptimalControl.jl).

In the OptimalControlProblems.jl package, you can find a list of optimal control problems modelled with JuMP and OptimalControl. There can be used for benchmarking.

**Content:**

- [List of the problems](@ref list-of-problems)
- [How to get a problem](@ref get-problem)
- [How to solve a problem](@ref solve-problem)
- [How to add a problem](@ref add-problem)

## Installation

To install OptimalControlProblems.jl, please [open Julia's interactive session (known as REPL)](https://docs.julialang.org/en/v1/manual/getting-started) and use the Julia package manager:

```julia
using Pkg
Pkg.add("OptimalControlProblems")
```

## Reproducibility

```@raw html
<details><summary>The documentation of this package was built using these direct dependencies,</summary>
```

```@example
using Pkg # hide
Pkg.status() # hide
```

```@raw html
</details>
```

```@raw html
<details><summary>and using this machine and Julia version.</summary>
```

```@example
using InteractiveUtils # hide
versioninfo() # hide
```

```@raw html
</details>
```

```@raw html
<details><summary>A more complete overview of all dependencies and their versions is also provided.</summary>
```

```@example
using Pkg # hide
Pkg.status(; mode = PKGMODE_MANIFEST) # hide
```

```@raw html
</details>
```

```@eval
using TOML
using Markdown
version = TOML.parse(read("../../Project.toml", String))["version"]
name = TOML.parse(read("../../Project.toml", String))["name"]
link_manifest = "https://github.com/SciML/" *
                name *
                ".jl/tree/gh-pages/v" *
                version *
                "/assets/Manifest.toml"
link_project = "https://github.com/SciML/" *
               name *
               ".jl/tree/gh-pages/v" *
               version *
               "/assets/Project.toml"
Markdown.parse("""You can also download the
[manifest]($link_manifest)
file and the
[project]($link_project)
file.
""")
```