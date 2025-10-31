# OptimalControlProblems.jl

The **OptimalControlProblems.jl** package is part of the [control-toolbox ecosystem](https://github.com/control-toolbox) and is independent of [OptimalControl.jl](https://control-toolbox.org/OptimalControl.jl). The control-toolbox ecosystem brings together Julia packages for mathematical control and its applications. Its purpose is to provide tools to model and solve optimal control problems governed by ordinary differential equations, using both direct and indirect methods, on CPU and GPU.  

If you would like to define and solve your own optimal control problem, please refer to the [OptimalControl.jl documentation](https://control-toolbox.org/OptimalControl.jl).

In **OptimalControlProblems.jl**, you will find a collection of optimal control problems modelled with JuMP and OptimalControl that can be used for benchmarking.

## Installation

To install **OptimalControlProblems.jl**, please [open Julia’s interactive session (known as the REPL)](https://docs.julialang.org/en/v1/manual/getting-started) and use the Julia package manager:

```julia
using Pkg
Pkg.add("OptimalControlProblems")
```

## Citing us

If you use OptimalControlProblems.jl in your work, please cite us:

> Caillau, J.-B., Cots, O., Gergaud, J., & Martinon, P. *OptimalControlProblems.jl: a collection of optimal control problems with ODE's in Julia*. [doi.org/10.5281/zenodo.17013180](https://doi.org/10.5281/zenodo.17013180)

or in bibtex format:

```bibtex
@software{OptimalControlProblems_jl,
author = {Caillau, Jean-Baptiste and Cots, Olivier and Gergaud, Joseph and Martinon, Pierre},
doi = {10.5281/zenodo.17013180},
license = {["MIT"]},
title = {{OptimalControlProblems.jl: a collection of optimal control problems with ODE's in Julia}},
url = {https://control-toolbox.org/OptimalControlProblems.jl}
}
```

## Contributing

If you think you found a bug or if you have a feature request / suggestion, feel free to open an [issue](https://github.com/control-toolbox/OptimalControlProblems.jl/issues). Before opening a pull request, please start an issue or a discussion on the topic. 

Contributions are welcomed, check out [how to contribute to a Github project](https://docs.github.com/en/get-started/exploring-projects-on-github/contributing-to-a-project). If it is your first contribution, you can also check [this first contribution tutorial](https://github.com/firstcontributions/first-contributions). You can find first good issues (if any 🙂) [here](https://github.com/control-toolbox/OptimalControlProblems.jl/contribute). You may find other packages to contribute to at the [control-toolbox organization](https://github.com/control-toolbox).

If you want to ask a question, feel free to start a discussion [here](https://github.com/orgs/control-toolbox/discussions). This forum is for general discussion about this repository and the [control-toolbox organization](https://github.com/control-toolbox).

!!! note

    If you want to add an application or a package to the control-toolbox ecosystem, please follow this [set up tutorial](https://github.com/orgs/control-toolbox/discussions/65).

## Credits (not exhaustive!)

- [BaptisteCbl](https://github.com/BaptisteCbl)
- [0Yassine0](https://github.com/0Yassine0)
- [Nico77310](https://github.com/Nico77310)
- [frapac](https://github.com/frapac)
- [BOCOP - A collection of examples](https://project.inria.fr/bocop/files/2017/05/Examples-BOCOP.pdf)
- [COPS: Large-Scale Optimization Problems](https://www.mcs.anl.gov/~more/cops) and [COPSBenchmark.jl](https://github.com/MadNLP/COPSBenchmark.jl)

## Reproducibility

```@setup main
using Pkg
using InteractiveUtils
using Markdown

# Download links for the benchmark environment
function _downloads_toml(DIR)
    link_manifest = joinpath("assets", DIR, "Manifest.toml")
    link_project = joinpath("assets", DIR, "Project.toml")
    return Markdown.parse("""
    You can download the exact environment used to build this documentation:
    - 📦 [Project.toml]($link_project) - Package dependencies
    - 📋 [Manifest.toml]($link_manifest) - Complete dependency tree with versions
    """)
end
```

```@example main
_downloads_toml(".") # hide
```

```@raw html
<details style="margin-bottom: 0.5em; margin-top: 1em;"><summary>ℹ️ Version info</summary>
```

```@example main
versioninfo() # hide
```

```@raw html
</details>
```

```@raw html
<details style="margin-bottom: 0.5em;"><summary>📦 Package status</summary>
```

```@example main
Pkg.status() # hide
```

```@raw html
</details>
```

```@raw html
<details style="margin-bottom: 0.5em;"><summary>📚 Complete manifest</summary>
```

```@example main
Pkg.status(; mode = PKGMODE_MANIFEST) # hide
```

```@raw html
</details>
```
