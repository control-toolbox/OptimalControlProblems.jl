# OptimalControlProblems.jl

```@meta
CurrentModule = OptimalControlProblems
```

**OptimalControlProblems.jl** is a collection of optimal control problems governed by ordinary
differential equations, for testing and benchmarking. Each problem comes with its description,
its parameters, a shared initial guess and a reference objective, and is written in one or more
**modelings**: the continuous problem with [OptimalControl.jl](https://control-toolbox.org/OptimalControl.jl/dev/),
a discretized problem with [JuMP](https://jump.dev). The package is part of the
[control-toolbox ecosystem](https://github.com/control-toolbox).

!!! note "Each modeling is an extension"
    The package alone lists the problems and their metadata. A modeling becomes available when
    its package is loaded: `using OptimalControl` for the OptimalControl modelings, `using JuMP`
    for JuMP. See [Installation](@ref installation).

## The modelings

```@eval
using Markdown
rows = ["| $(a.title) | `$(nameof(s.backend))()` | $(a.summary) |" for (s, a) in Main.modelings()]
Markdown.parse("""
| Modeling | Backend | What you get |
| :--- | :--- | :--- |
$(join(rows, "\n"))
""")
```

Every problem written in two modelings is checked to give the same nonlinear program in both,
and the solves of every modeling are compared to one reference objective. Each
[problem page](@ref problems-overview) shows the modelings side by side.

## Quick start

```julia
using OptimalControl, OptimalControlProblems
using NLPModelsIpopt

pb = beam(OptimalControlBackend())   # the continuous problem
sol = solve(
    model(pb), :collocation, :adnlp, :ipopt;
    init=initial_guess(pb), grid_size=500, scheme=:trapeze,
)
objective(sol)
```

## Where to go next

- **[Your first problem](@ref first-problem)**: get a problem, solve it, check the result.
- **[Guided tour](@ref guided-tour)**: the collection, the metadata, the modelings.
- **User guide**: [get a problem](@ref get-problem), solve it with
  [OptimalControl](@ref solve-optimalcontrol) or [JuMP](@ref solve-jump),
  [choose the initial guess](@ref initial-guess), [compare modelings](@ref compare-modelings),
  [plot a solution](@ref plot-solution).
- **[Problems](@ref problems-overview)**: every problem, with its modelings and results.

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

If you think you found a bug or if you have a feature request / suggestion, feel free to open an [issue](https://github.com/control-toolbox/OptimalControlProblems.jl/issues). Before opening a pull request, please start an issue or a discussion on the topic. To add a problem, see the [Developers](@ref developers) page.

If you want to ask a question, feel free to start a discussion [here](https://github.com/orgs/control-toolbox/discussions). This forum is for general discussion about this repository and the [control-toolbox organization](https://github.com/control-toolbox).

## Credits (not exhaustive!)

- [BaptisteCbl](https://github.com/BaptisteCbl)
- [0Yassine0](https://github.com/0Yassine0)
- [Nico77310](https://github.com/Nico77310)
- [frapac](https://github.com/frapac)
- [AmielMetier](https://github.com/AmielMetier)
- [HediChennoufi](https://github.com/HediChennoufi)
- [BOCOP - A collection of examples](https://project.inria.fr/bocop/files/2017/05/Examples-BOCOP.pdf)
- [COPS: Large-Scale Optimization Problems](https://www.mcs.anl.gov/~more/cops) and [COPSBenchmark.jl](https://github.com/MadNLP/COPSBenchmark.jl)

## Reproducibility

```@setup main
using Pkg
using InteractiveUtils
using Markdown
```

You can download the exact environment used to build this documentation:

```@raw html
<ul>
  <li>📦 <a href="./assets/Project.toml" download>Project.toml</a>: package dependencies</li>
  <li>📋 <a href="./assets/Manifest.toml" download>Manifest.toml</a>: complete dependency tree with versions</li>
</ul>
```

::: details ℹ️ Version info

```@example main
versioninfo() # hide
```

:::

::: details 📦 Package status

```@example main
Pkg.status() # hide
```

:::

::: details 📚 Complete manifest

```@example main
Pkg.status(; mode = PKGMODE_MANIFEST) # hide
```

:::
