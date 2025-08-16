# [Introduction](@id problems-introduction)

An optimal control problem (OCP) with fixed initial and final times can be described as minimising the cost functional

```math
g(x(t_0), x(t_f)) + \int_{t_0}^{t_f} f^{0}(t, x(t), u(t))~\mathrm{d}t
```

where the state $x$ and the control $u$ are functions of time $t$, subject for $t \in [t_0, t_f]$ to the differential constraint

```math
   \dot{x}(t) = f(t, x(t), u(t))
```

and other constraints such as

```math
\begin{array}{llcll}
x_{\mathrm{lower}} & \le & x(t)              & \le & x_{\mathrm{upper}}, \\
u_{\mathrm{lower}} & \le & u(t)              & \le & u_{\mathrm{upper}}, \\
c_{\mathrm{lower}} & \le & c(t, x(t), u(t))  & \le & c_{\mathrm{upper}}, \\
b_{\mathrm{lower}} & \le & b(x(t_0), x(t_f)) & \le & b_{\mathrm{upper}}.
\end{array}
```

!!! note
    The initial time $t_0$ and the final time $t_f$ may also be free. More generally, additional variables can be introduced and optimised under further constraints.

The so-called **direct approach** transforms the infinite-dimensional optimal control problem (OCP) into a finite-dimensional optimisation problem (NLP). This is achieved by discretising time, typically with Runge–Kutta methods applied to the state, control variables, and dynamics equation. These methods are usually less precise than indirect methods based on [Pontryagin’s Maximum Principle](https://en.wikipedia.org/w/index.php?title=Pontryagin's_maximum_principle&oldid=1160355192), but they are more robust with respect to initialisation. They are also easier to apply, which explains their widespread use in industrial applications.

In the **OptimalControlProblems** package, each OCP is discretised using the trapezoidal rule on a uniform grid:

```math
\begin{array}{lclr}
t \in [t_0,t_f]   & \to & \{t_0, \ldots, t_N=t_f\}                    & \\[0.5em]
x(\cdot),\, u(\cdot) & \to & X=\{x_0, \ldots, x_N, u_0, \ldots, u_N\} & \\[1em]
\hline
\\
\text{step} & \to & \displaystyle h = \frac{t_f-t_0}{N}  & \\[0.5em]
\text{criterion} & \to & \displaystyle g(x_0, x_N) + 
    \frac{h}{2} \sum_{i=1}^{N} \left( f^0(t_i, x_i, u_i) + f^0(t_{i-1}, x_{i-1}, u_{i-1}) \right)   & \\[1em]
\text{dynamics}  & \to & \displaystyle x_{i} = x_{i-1} + 
    \frac{h}{2} \left( f(t_i, x_i, u_i) + f(t_{i-1}, x_{i-1}, u_{i-1}) \right),                     & i = 1:N \\[1em]
\text{state constraints}    & \to & x_{\mathrm{lower}} \le x_i              \le x_{\mathrm{upper}}, & i = 0:N \\[1em]
\text{control constraints}  & \to & u_{\mathrm{lower}} \le u_i              \le u_{\mathrm{upper}}, & i = 0:N \\[1em]
\text{path constraints}     & \to & c_{\mathrm{lower}} \le c(t_i, x_i, u_i) \le c_{\mathrm{upper}}, & i = 0:N \\[1em]
\text{boundary constraints} & \to & b_{\mathrm{lower}} \le b(x_0, x_N)      \le b_{\mathrm{upper}}  & 
\end{array}
```

We therefore obtain a nonlinear programming problem (NLP) on the discretised state and control variables of the general form:

```math
\text{(NLP)} \quad \left\{
\begin{array}{lr}
\min \ F(X) \\[1em]
X_{\mathrm{lower}} \le X \le X_{\mathrm{upper}}\\[0.5em]
C_{\mathrm{lower}} \le C(X) \le C_{\mathrm{upper}}
\end{array}
\right.
```

## Models

Each optimal control problem in the **OptimalControlProblems** package is modelled both in [JuMP](https://jump.dev/JuMP.jl) and in [OptimalControl](https://control-toolbox.org/OptimalControl.jl). The problem definitions are stored in the [OptimalControlProblems.jl/ext](https://github.com/control-toolbox/OptimalControlProblems.jl/tree/main/ext) directory:

- JuMP models are stored in the [JuMPModels](https://github.com/control-toolbox/OptimalControlProblems.jl/tree/main/ext/JuMPModels) directory. These codes implement the NLP problem directly.
- OptimalControl models are stored in the [OptimalControlModels](https://github.com/control-toolbox/OptimalControlProblems.jl/tree/main/ext/OptimalControlModels) directory. These codes represent the OCP, and the discretisation is handled by the package. The resulting NLP is represented by an [`ADNLPModels.ADNLPModel`](@extref), which provides automatic differentiation (AD)-based models following the [NLPModels.jl](https://github.com/JuliaSmoothOptimizers/NLPModels.jl) API.

To get the list of available problems, first import the package:

```@example main
using OptimalControlProblems
```

Then, call:

```@example main
available_problems()
```

## [Metadata](@id problems-introduction-metadata)

For each problem, additional data is provided in the [MetaData](https://github.com/control-toolbox/OptimalControlProblems.jl/tree/main/ext/MetaData) directory:

```@docs; canonical=false
OptimalControlProblems.metadata
```

To list all metadata, use `OptimalControlProblems.metadata`.  
To access the metadata of a specific problem, for example `chain`, run:

```@example main
OptimalControlProblems.metadata[:chain]
```

## List of the problems

```@contents
Pages = Main.PROBLEMS_PAGES
Depth = 1
```
