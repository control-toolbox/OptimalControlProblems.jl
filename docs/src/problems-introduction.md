# [Introduction](@id problems-introduction)

An optimal control problem (OCP) with fixed initial and final times, can be described as minimising the cost functional

```math
g(x(t_0), x(t_f)) + \int_{t_0}^{t_f} f^{0}(t, x(t), u(t))~\mathrm{d}t
```

where the state $x$ and the control $u$ are functions of time $t$ subject, for $t \in [t_0, t_f]$, to the differential constraint

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

    The initial time $t_0$, the final time $t_f$ can be free. More generally, we can have a set of variables to optimise under some additional constraints.

The so-called direct approach transforms the infinite dimensional optimal control problem (OCP) into a finite dimensional optimization problem (NLP). This is done by a discretization in time by Runge-Kutta methods applied to the state and control variables, as well as the dynamics equation. These methods are usually less precise than indirect methods based on [Pontryagin’s Maximum Principle](https://en.wikipedia.org/w/index.php?title=Pontryagin's_maximum_principle&oldid=1160355192), but more robust with respect to the initialization. Also, they are more straightforward to apply, hence their wide use in industrial applications. We refer the reader to [^1] and [^2] for more details on direct transcription methods and NLP algorithms.

[^1]: J. T. Betts. Practical methods for optimal control using nonlinear programming. Society for Industrial and Applied Mathematics (SIAM), Philadelphia, PA, 2001.

[^2]: J. Nocedal and S.J. Wright. Numerical optimization. Springer-Verlag, New York, 1999.****

In OptimalControlProblems package, each optimal control problem is discretised by the trapezoidal rule on a uniform grid:

```math
\begin{array}{lclr}
t \in [t_0,t_f]   & \to & \{t_0, \ldots, t_N=t_f\}                    & \\[0.5em]
x(\cdot),\, u(\cdot) & \to & X=\{x_0, \ldots, x_N, u_0, \ldots, u_N\} & \\[1em]
\hline
\\
\text{step} & \to & \displaystyle h = \frac{t_f-t_0}{N}  & \\[0.5em]
\text{criterion} & \to & \displaystyle g(x_0, x_N) + 
    \frac{h}{2} \sum_{i=1}^{N} \left( f^0(t_i, x_i, u_i) + f^0(t_{i-1}, x_{i-1}, u_{i-1}) \right)   & \\[0.5em]
\text{dynamics}  & \to & \displaystyle x_{i} = x_{i-1} + 
    \frac{h}{2} \left( f(t_i, x_i, u_i) + f(t_{i-1}, x_{i-1}, u_{i-1}) \right),                     & i = 1:N \\[0.5em]
\text{state constraints}    & \to & x_{\mathrm{lower}} \le x_i              \le x_{\mathrm{upper}}, & i = 0:N \\[0.5em]
\text{control constraints}  & \to & u_{\mathrm{lower}} \le u_i              \le u_{\mathrm{upper}}, & i = 0:N \\[0.5em]
\text{path constraints}     & \to & c_{\mathrm{lower}} \le c(t_i, x_i, u_i) \le c_{\mathrm{upper}}, & i = 0:N \\[0.5em]
\text{boundary constraints} & \to & b_{\mathrm{lower}} \le b(x_0, x_N)      \le b_{\mathrm{upper}}  & 
\end{array}
```

We therefore obtain a nonlinear programming problem (NLP) on the discretized state and control variables of the general form:

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

Each optimal control problem of OptimalControlProblems package is modelled in [JuMP](https://jump.dev/JuMP.jl) and [OptimalControl](https://control-toolbox.org/OptimalControl.jl). The codes of the problems are stored in [OptimalControlProblems.jl/ext](https://github.com/control-toolbox/OptimalControlProblems.jl/tree/main/ext) directory:

- JuMP models are stored in [JuMPModels](https://github.com/control-toolbox/OptimalControlProblems.jl/tree/main/ext/JuMPModels) directory. The codes of the JuMP models correspond to the NLP problem.
- OptimalControl models are stored in [OptimalControlModels](https://github.com/control-toolbox/OptimalControlProblems.jl/tree/main/ext/OptimalControlModels) directory. The codes of the OptimalControl models represent the OCP. In this case, the discretisation is made by the package. The NLP problem is modelled by a [`ADNLPModels.ADNLPModel`](@extref). It provides automatic differentiation (AD)-based models that follows the [NLPModels.jl](https://github.com/JuliaSmoothOptimizers/NLPModels.jl) API.

To get the list of available problems, first import the package:

```@example main
using OptimalControlProblems
```

Then, you can call the following function:

```@example main
available_problems()
```

## [Metadata](@id problems-introduction-metadata)

For each problem, we provide also the following data in [MetaData](https://github.com/control-toolbox/OptimalControlProblems.jl/tree/main/ext/MetaData) directory:

- `name::String`: problem name.
- `N::Int`: default number of discretization points.
- `minimize::Bool`: true or false depending on whether we minimise or maximise the objective function.
- `state_name::Vector{String}`: the names of the components of the state.
- `costate_name::Vector{String}`: the names of the differential constraints associated to each component of the costate.
- `control_name::Vector{String}`: the names of the components of the control.
- `time::Tuple{String, String, Union{Int, Nothing}}`: `time` is of the form `(type, name, value)` where:
  - `type` is either `final_time` or `step` depending on how the problem is modelled. Either the final time or the time step is a decision variable. If the final time is fixed, then `type="final_time"`.
  - `name` is the name of the final time variable.
  - `value` is either the value of the final time or `nothing` if it is free. If the final time is fixed, then it is simply a parameter while if it is free, then it is one of the decision variable. If `type="step"`, then `value` is the value of the time step (assuming the grid is uniform).

To get the list of metadata, you can get access to `OptimalControlProblems.metadata`. To access the metadata of a specific problem, for instance `chain`, you can execute the following command:

```@example main
OptimalControlProblems.metadata[:chain]
```

## List of the problems

```@contents
Pages = Main.PROBLEMS_PAGES
Depth = 1
```