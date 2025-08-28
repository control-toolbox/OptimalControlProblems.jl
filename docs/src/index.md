# OptimalControlProblems.jl

The **OptimalControlProblems.jl** package is part of the [control-toolbox ecosystem](https://github.com/control-toolbox) and is independent of [OptimalControl.jl](https://control-toolbox.org/OptimalControl.jl). The control-toolbox ecosystem brings together Julia packages for mathematical control and its applications. Its purpose is to provide tools to model and solve optimal control problems governed by ordinary differential equations, using both direct and indirect methods, on CPU and GPU.  

If you would like to define and solve your own optimal control problem, please refer to the [OptimalControl.jl documentation](https://control-toolbox.org/OptimalControl.jl).

In **OptimalControlProblems.jl**, you will find a collection of optimal control problems modelled with JuMP and OptimalControl. These problems can be used for benchmarking.

## Installation

To install **OptimalControlProblems.jl**, please [open Julia’s interactive session (known as the REPL)](https://docs.julialang.org/en/v1/manual/getting-started) and use the Julia package manager:

```julia
using Pkg
Pkg.add("OptimalControlProblems")
```

## Credits (not exhaustive!)

- [Nico77310](https://github.com/Nico77310)
- [0Yassine0](https://github.com/0Yassine0)
- [frapac](https://github.com/frapac)
- [BaptisteCbl](https://github.com/BaptisteCbl)
- [COPS: Large-Scale Optimization Problems](https://www.mcs.anl.gov/~more/cops) and [COPSBenchmark.jl](github.com/MadNLP/COPSBenchmark.jl)
- [BOCOP - A collection of examples](https://project.inria.fr/bocop/files/2017/05/Examples-BOCOP.pdf)

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
link_manifest = "https://github.com/control-toolbox/" *
                name *
                ".jl/tree/gh-pages/v" *
                version *
                "/assets/Manifest.toml"
link_project = "https://github.com/control-toolbox/" *
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
