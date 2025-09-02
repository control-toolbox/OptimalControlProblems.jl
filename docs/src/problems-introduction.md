# [Introduction](@id problems-introduction)

## Discretise optimal control problems

An optimal control problem (OCP) with fixed initial and final times can be described as minimising the cost functional (in Bolza form)

```math
J(x, u) = g(x(t_0), x(t_f)) + \int_{t_0}^{t_f} f^{0}(t, x(t), u(t))~\mathrm{d}t
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

If $g = 0$, the cost is said to be in **Lagrange form**; if $f^0 = 0$, it is in **Mayer form**.

!!! note
    The initial time $t_0$ and the final time $t_f$ may also be free, that is part of the optimisation variables:
    ```math
    J(x, u, t_0, t_f) \to \min.
    ```
    More generally, additional variables can be introduced:
    ```math
    J(x, u, v) \to \min,
    ```
    with $v \in \mathbb{R}^k$ the vector of the $k$ variables (it may contain $t_0$, $t_f$), under further constraints:
    ```math
    v_{\mathrm{lower}} \le v \le v_{\mathrm{upper}}.
    ```

The so-called **direct approach** transforms the infinite-dimensional optimal control problem (OCP) into a finite-dimensional optimisation problem (NLP). This is achieved by discretising time, typically with Runge–Kutta methods applied to the state, control variables, and dynamics equation. These methods are usually less precise than indirect methods based on [Pontryagin’s Maximum Principle](https://en.wikipedia.org/w/index.php?title=Pontryagin's_maximum_principle&oldid=1160355192), but they are more robust with respect to initialisation. They are also easier to apply, which explains their widespread use in industrial applications.

In the **OptimalControlProblems** package, every OCP is discretized—at minimum—using the trapezoidal rule on a uniform grid:

```math
\begin{array}{lclr}
t \in [t_0,t_f]   & \to & t_0 < t_1 < \dots < t_N=t_f, \text{ with } t_{i}-t_{i-1} = \frac{t_f-t_0}{N}, & i = 1:N & \\[0.5em]
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

## JuMP and OptimalControl models

Each optimal control problem in the **OptimalControlProblems** package is modelled both in [JuMP](https://jump.dev/JuMP.jl) and in [OptimalControl](https://control-toolbox.org/OptimalControl.jl). The problem definitions are stored in the [OptimalControlProblems.jl/ext](https://github.com/control-toolbox/OptimalControlProblems.jl/tree/main/ext) directory:

- JuMP models are stored in the [OptimalControlProblems.jl/ext/JuMPModels](https://github.com/control-toolbox/OptimalControlProblems.jl/tree/main/ext/JuMPModels) directory. These codes implement the NLP problem directly.
- OptimalControl models are located in the [OptimalControlProblems.jl/ext/OptimalControlModels](https://github.com/control-toolbox/OptimalControlProblems.jl/tree/main/ext/OptimalControlModels) directory.  These files define the OCPs themselves, while discretisation is handled by the package through [`CTDirect.direct_transcription`](@extref). Each model returns a [`CTDirect.DOCP`](@extref) (Discretised Optimal Control Problem), which may be thought of as combining the continuous OCP with its corresponding NLP formulation.

For more specific details about the problems, see the following pages. We provide descriptions of the optimal control problems and compare the different models.

```@contents
Pages = Main.PROBLEMS_PAGES
Depth = 1
```

## [Problems metadata](@id problems-introduction-metadata)

For each problem, additional data is provided in the [MetaData](https://github.com/control-toolbox/OptimalControlProblems.jl/tree/main/ext/MetaData) directory:

```@docs; canonical=false
metadata
```

To list all metadata, use `metadata`. To access the metadata of a specific problem, for example `chain`, run:

```@example main
using OptimalControlProblems
metadata[:chain]
```

## Problems characteristics

To get the list of available problems, call the [`problems`](@ref) method.

```@example main
problems()
```

We detail below the characteristics of the optimal control problems and their associated nonlinear programming problems. 

!!! note "Code for retrieving the data of each OCP and NLP"

    ```@raw html
    <details><summary>Click to unfold and get the code to get the data.</summary>
    ```

    ```@example main
    using NLPModels                 # to get the number of variables and constraints
    import DataFrames: DataFrame    # to store data
    using OptimalControl

    data_ocp = DataFrame(           # to store data of the OCPs
        Problem=Symbol[],
        State=Int[],
        Control=Int[],
        Variable=Int[],
        Cost=Symbol[],
        FinalTime=Symbol[],
        Constraints=String[],
    )

    data_nlp = DataFrame(           # to store data of the NLPs
        Problem=Symbol[],
        Steps=Int[],
        Variables=Int[],
        Constraints=Int[],
    )

    for problem in problems()

        #
        docp = eval(problem)(OptimalControlBackend())
        nlp = nlp_model(docp)
        ocp = ocp_model(docp)

        #
        cost = if has_mayer_cost(ocp) && has_lagrange_cost(ocp)
            :Bolza
        elseif has_mayer_cost(ocp)
            :Mayer
        else
            :Lagrange
        end

        #
        final_time = has_fixed_final_time(ocp) ? :fixed : :free

        #
        using CTModels # these functions should be exported by OptimalControl
        dim_state_cons_box = CTModels.dim_state_constraints_box(ocp)
        dim_control_cons_box = CTModels.dim_control_constraints_box(ocp)
        dim_variable_cons_box = CTModels.dim_variable_constraints_box(ocp)
        dim_path_cons_nl = CTModels.dim_path_constraints_nl(ocp)
        dim_boundary_cons_nl = CTModels.dim_boundary_constraints_nl(ocp)
        constraints = ""
        constraints *= dim_state_cons_box    > 0 ? "x" : ""
        constraints *= dim_control_cons_box  > 0 ? (isempty(constraints) ? "" : ", ") * "u" : ""
        constraints *= dim_variable_cons_box > 0 ? (isempty(constraints) ? "" : ", ") * "v" : ""
        constraints *= dim_path_cons_nl      > 0 ? (isempty(constraints) ? "" : ", ") * "c" : ""
        constraints *= dim_boundary_cons_nl  > 0 ? (isempty(constraints) ? "" : ", ") * "b" : ""
        constraints = if !isempty(constraints)
            "(" * constraints * ")"
        end

        #
        push!(data_ocp,
            (
                Problem=problem,
                State=state_dimension(ocp),
                Control=control_dimension(ocp),
                Variable=variable_dimension(ocp),
                Cost=cost,
                FinalTime=final_time,
                Constraints=constraints,
            )
        )

        #
        N = metadata[problem][:N] # get default number of steps

        push!(data_nlp,
            (
                Problem=problem,
                Steps=N,
                Variables=get_nvar(nlp),
                Constraints=get_ncon(nlp),
            )
        )
    end
    ```

    ```@raw html
    </details>
    ```

For each optimal control problem, we provide the dimensions of the state, control, and variable. We also specify the type of objective function (Mayer, Lagrange, or Bolza), whether the final time is free or fixed, and whether there are constraints on the state (`x`), control (`u`), variable (`v`), path (`c`—from *chemin* in French, since `p` is reserved for the costate), or boundary (`b`).

```@example main
data_ocp
```

```@raw html
</br>
```
For each nonlinear programming problem, we give the default number of steps, the number of variables and the numbers of constraints.

```@example main
data_nlp
```