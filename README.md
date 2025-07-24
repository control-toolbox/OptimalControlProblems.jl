# OptimalControlProblems.jl

[ci-img]: https://github.com/control-toolbox/OptimalControlProblems.jl/actions/workflows/CI.yml/badge.svg?branch=main
[ci-url]: https://github.com/control-toolbox/OptimalControlProblems.jl/actions/workflows/CI.yml?query=branch%3Amain

[co-img]: https://codecov.io/gh/control-toolbox/OptimalControlProblems.jl/branch/main/graph/badge.svg?token=YM5YQQUSO3
[co-url]: https://codecov.io/gh/control-toolbox/OptimalControlProblems.jl

[doc-dev-img]: https://img.shields.io/badge/docs-dev-8A2BE2.svg
[doc-dev-url]: https://control-toolbox.org/OptimalControlProblems.jl/dev/

[doc-stable-img]: https://img.shields.io/badge/docs-stable-blue.svg
[doc-stable-url]: https://control-toolbox.org/OptimalControlProblems.jl/stable/

[release-img]: https://img.shields.io/github/v/release/control-toolbox/OptimalControlProblems.jl.svg
[release-url]: https://github.com/control-toolbox/OptimalControlProblems.jl/releases

[pkg-eval-img]: https://img.shields.io/badge/Julia-package-purple
[pkg-eval-url]: https://juliahub.com/ui/Packages/General/OptimalControlProblems

[licence-img]: https://img.shields.io/badge/License-MIT-yellow.svg
[licence-url]: https://github.com/control-toolbox/OptimalControlProblems.jl/blob/master/LICENSE

[blue-img]: https://img.shields.io/badge/code%20style-blue-4495d1.svg
[blue-url]: https://github.com/JuliaDiff/BlueStyle

This repo is part of the [control-toolbox ecosystem](https://github.com/control-toolbox).
The control-toolbox ecosystem gathers Julia packages for mathematical control and applications. The root package is [OptimalControl.jl](https://github.com/control-toolbox/OptimalControl.jl) which aims to provide tools to model and solve optimal control problems with ordinary differential equations by direct and indirect methods, both on CPU and GPU.

[![doc OptimalControl.jl](https://img.shields.io/badge/Documentation-OptimalControl.jl-blue)](http://control-toolbox.org/OptimalControl.jl)

| **Name**          | **Badge**         |
:-------------------|:------------------|
| Documentation     | [![Documentation][doc-stable-img]][doc-stable-url] [![Documentation][doc-dev-img]][doc-dev-url]                   | 
| Code Status       | [![Build Status][ci-img]][ci-url] [![Covering Status][co-img]][co-url] [![pkgeval][pkg-eval-img]][pkg-eval-url] [![Code Style: Blue][blue-img]][blue-url]  |
| Licence           | [![License: MIT][licence-img]][licence-url]   |
| Release           | [![Release][release-img]][release-url]        |

## Installation

To install OptimalControlProblems.jl please 
<a href="https://docs.julialang.org/en/v1/manual/getting-started/">open Julia's interactive session (known as REPL)</a> 
and press <kbd>]</kbd> key in the REPL to use the package mode, then add the package:

```julia
julia> ]
pkg> add OptimalControlProblems
```

> [!TIP]
> If you are new to Julia, please follow this [guidelines](https://github.com/orgs/control-toolbox/discussions/64).

## Contributing

[issue-url]: https://github.com/control-toolbox/OptimalControlProblems.jl/issues
[first-good-issue-url]: https://github.com/control-toolbox/OptimalControlProblems.jl/contribute

If you think you found a bug or if you have a feature request / suggestion, feel free to open an [issue][issue-url].
Before opening a pull request, please start an issue or a discussion on the topic. 

Contributions are welcomed, check out [how to contribute to a Github project](https://docs.github.com/en/get-started/exploring-projects-on-github/contributing-to-a-project). 
If it is your first contribution, you can also check [this first contribution tutorial](https://github.com/firstcontributions/first-contributions).
You can find first good issues (if any 🙂) [here][first-good-issue-url]. You may find other packages to contribute to at the [control-toolbox organization](https://github.com/control-toolbox).

If you want to ask a question, feel free to start a discussion [here](https://github.com/orgs/control-toolbox/discussions). This forum is for general discussion about this repository and the [control-toolbox organization](https://github.com/control-toolbox).

>[!NOTE]
> If you want to add an application or a package to the control-toolbox ecosystem, please follow this [set up tutorial](https://github.com/orgs/control-toolbox/discussions/65).
