# JuMP transcription

This page is for users and developers who want to know **exactly which nonlinear program
(NLP)** a JuMP model of this package is, and how the model stores what is needed to read
the state, the control and the costate back. It is not needed to use the package.

The OptimalControl modelings return the continuous problem; the discretization is chosen
in `solve` and done by CTDirect. The JuMP modeling has no such layer: each JuMP model
**is** an NLP. To compare JuMP and OptimalControl fairly, the JuMP models reproduce the
NLP that CTDirect builds, scheme by scheme.

## The continuous problem

Every problem is written in Bolza form, with state ``x(t) \in \mathbb{R}^n``, control
``u(t) \in \mathbb{R}^m`` and a variable ``v \in \mathbb{R}^q`` (e.g. a free final time):

```math
\begin{aligned}
\min_{x,\, u,\, v} \quad & g\big(x(t_0), x(t_f), v\big) + \int_{t_0}^{t_f} f^0\big(t, x(t), u(t), v\big)\, \mathrm{d}t \\
\text{s.t.} \quad & \dot x(t) = f\big(t, x(t), u(t), v\big), \quad t \in [t_0, t_f], \\
& c_l \le c\big(t, x(t), u(t), v\big) \le c_u, \quad t \in [t_0, t_f] \quad \text{(path constraints)}, \\
& b_l \le b\big(x(t_0), x(t_f), v\big) \le b_u \quad \text{(boundary conditions)}, \\
& x_l \le x(t) \le x_u, \quad u_l \le u(t) \le u_u, \quad v_l \le v \le v_u \quad \text{(box constraints)}.
\end{aligned}
```

``t_0`` and ``t_f`` are numbers or components of ``v`` (free initial or final time).

## The grid

With ``N`` = `grid_size`, the grid is uniform:

```math
h = \frac{t_f - t_0}{N}, \qquad t_i = t_0 + i\,h, \quad i = 0, \dots, N.
```

When ``t_f`` is free, ``h`` depends on ``v``: the dynamics are then nonlinear in ``v``
even for a linear system.

Indices start at ``0`` on this page, as the JuMP containers do (`x₁[0:N]`). CTDirect
counts from ``1``: its ``X_1, \dots, X_{N+1}`` are ``X_0, \dots, X_N`` here.

## Unknowns and number of controls

The state is discretized at the ``N+1`` nodes: ``X_0, \dots, X_N \in \mathbb{R}^n``. The
number of controls **depends on the scheme**:

| Scheme | `scheme` | Controls | Number of unknowns |
| --- | --- | --- | --- |
| Trapezoidal rule | `:trapeze` | ``U_0, \dots, U_N`` (one per node) | ``(N+1)(n+m) + q`` |
| Explicit Euler | `:euler` | ``U_0, \dots, U_{N-1}`` (one per step) | ``(N+1)n + Nm + q`` |
| Implicit Euler | `:euler_implicit` | ``U_0, \dots, U_{N-1}`` (one per step) | ``(N+1)n + Nm + q`` |
| Midpoint rule | `:midpoint` | ``U_0, \dots, U_{N-1}`` (one per step) | ``(N+1)n + Nm + q`` |

For the one-step schemes, ``U_i`` is the control on the step ``[t_i, t_{i+1}]``: there is
no control "at ``t_f``". With the trapezoidal rule the control is a value at each node.
Everything that reads the controls back (accessors, plots, the construction of a
solution) has to know which case applies.

The scheme of a JuMP model is chosen when it is built, with the names of
`OptimalControl.Collocation`; the default is the trapezoidal rule:

```julia
model = beam(JuMPBackend(); grid_size=100, scheme=:midpoint)
model[:scheme]      # :midpoint
```

## Dynamics and cost, scheme by scheme

Write ``f_i = f(t_i, X_i, U_i, v)`` and ``f^0_i`` likewise. For ``i = 0, \dots, N-1``:

| Scheme | Dynamics | Lagrange cost |
| --- | --- | --- |
| `:trapeze` | ``X_{i+1} = X_i + \frac{h}{2}\,(f_i + f_{i+1})`` | ``\frac{h}{2} \sum_{i=0}^{N-1} (f^0_i + f^0_{i+1})`` |
| `:euler` | ``X_{i+1} = X_i + h\, f(t_i, X_i, U_i, v)`` | ``h \sum_{i=0}^{N-1} f^0(t_i, X_i, U_i, v)`` |
| `:euler_implicit` | ``X_{i+1} = X_i + h\, f(t_{i+1}, X_{i+1}, U_i, v)`` | ``h \sum_{i=0}^{N-1} f^0(t_{i+1}, X_{i+1}, U_i, v)`` |
| `:midpoint` | ``X_{i+1} = X_i + h\, f\big(t_i + \frac{h}{2}, \frac{X_i + X_{i+1}}{2}, U_i, v\big)`` | ``h \sum_{i=0}^{N-1} f^0\big(t_i + \frac{h}{2}, \frac{X_i + X_{i+1}}{2}, U_i, v\big)`` |

The Lagrange cost is added to the objective directly, by the quadrature of the scheme: no
extra state is introduced. The Mayer term is ``g(X_0, X_N, v)``.

## The schemes as Butcher tableaux

The four schemes are Runge–Kutta methods, each given by its Butcher tableau
``(A, b, c)`` with ``s`` stages. A step reads

```math
Z_j = X_i + h \sum_{k=1}^{s} a_{jk}\, f(t_i + c_k h, Z_k, \cdot), \qquad
X_{i+1} = X_i + h \sum_{j=1}^{s} b_j\, f(t_i + c_j h, Z_j, \cdot),
```

and the Lagrange cost uses the same quadrature, ``h \sum_i \sum_j b_j f^0_{ij}``.

| Scheme | ``A`` | ``b`` | ``c`` |
| :--- | :--- | :--- | :--- |
| `:euler` | ``[0]`` | ``[1]`` | ``[0]`` |
| `:euler_implicit` | ``[1]`` | ``[1]`` | ``[1]`` |
| `:midpoint` | ``[\frac12]`` | ``[1]`` | ``[\frac12]`` |
| `:trapeze` | ``\begin{bmatrix} 0 & 0 \\ \frac12 & \frac12 \end{bmatrix}`` | ``[\frac12, \frac12]`` | ``[0, 1]`` |

The JuMP models have **no stage unknowns**: each stage is written with the nodes of its
step, ``Z_j = X_i + \theta_j (X_{i+1} - X_i)``. For a one-stage tableau,
``\theta = a_{11} / b_1``, which gives ``X_i``, ``X_{i+1}`` and ``\frac{X_i + X_{i+1}}{2}``
for the explicit Euler, implicit Euler and midpoint rules; for a tableau of several stages,
every stage must be a node of the step. A tableau with a stage inside the step
(Hermite–Simpson, Gauss–Legendre, Radau) would need stage unknowns and is not supported.

Two properties of the tableau decide the rest:

- **stiffly accurate**: the last row of ``A`` is ``b`` and ``c_s = 1``, so the last stage
  is ``X_{i+1}``;
- **explicit first stage**: the first row of ``A`` is zero and ``c_1 = 0``, so the first
  stage is ``X_i``.

A tableau with both is **FSAL** (*First Same As Last*): the last stage of a step is the
first stage of the next one. Among the four schemes, only the trapezoidal rule is FSAL.

| Scheme | Stiffly accurate | Explicit first stage | FSAL |
| :--- | :--- | :--- | :--- |
| `:euler` | no | yes | no |
| `:euler_implicit` | yes | no | no |
| `:midpoint` | no | no | no |
| `:trapeze` | yes | yes | yes |

With FSAL, the dynamics and the Lagrange integrand are evaluated **once per node**,
``N+1`` evaluations instead of ``2N``, and the control of a stage is the control of its
node, shared by the two steps around it: hence ``N+1`` controls. Without FSAL, each step
has its own control: ``N`` controls. The node used by the path constraints follows: the
control of the step that starts at the node, except for a scheme whose only stage ends the
step (implicit Euler), which takes the control of the step that ends at the node.

These properties are computed from the tableau, not declared, in
`ext/JuMPModels/schemes/tableaux.jl`; the names follow the roadmap of CTDirect
([CTDirect.jl#610](https://github.com/control-toolbox/CTDirect.jl/issues/610)), where the
same tableaux, traits and specializations are planned.

## Constraints

**Box constraints** on the state, the control and the variable are **bounds of the
unknowns**, not constraints: ``x_l \le X_i \le x_u``, ``u_l \le U_i \le u_u``,
``v_l \le v \le v_u``.

**Path constraints** are imposed at the ``N+1`` nodes, each with the control "at" that
node:

| Scheme | Node ``i`` uses |
| --- | --- |
| `:trapeze` | ``U_i`` |
| `:euler`, `:midpoint` | ``U_i`` for ``i < N``, and ``U_{N-1}`` at the last node |
| `:euler_implicit` | ``U_0`` at the first node, and ``U_{i-1}`` for ``i \ge 1`` |

**Boundary conditions** ``b_l \le b(X_0, X_N, v) \le b_u`` are constraints, even when
they fix a component of the state (the variable is not fixed).

!!! note "Exa and the number of controls"

    Under the `:exa` modeler of OptimalControl, the controls are declared at the ``N+1``
    nodes for every scheme. For `:euler`, `:midpoint` and `:euler_implicit` the Exa NLP
    therefore has ``m`` more unknowns than CTDirect's, used only by their bounds and by
    one extreme path constraint. See
    [#284](https://github.com/control-toolbox/OptimalControlProblems.jl/issues/284).

## Order of the unknowns and of the constraints

The order of the unknowns and of the constraints changes the sparsity pattern seen by the
linear solver. It is chosen when the model is built, with `ordering`:

- **`:interleaved`** (default), CTDirect's order:
  - **unknowns**: node by node, the state then the control while there is one,
    ``X_0, U_0, X_1, U_1, \dots, X_N, U_N`` (trapezoidal rule) or
    ``X_0, U_0, \dots, X_{N-1}, U_{N-1}, X_N`` (one control per step), then ``v``;
  - **constraints**: for each step ``i = 0, \dots, N-1``, the dynamics of the step then
    the path constraints at node ``i``; then the path constraints at node ``N``; then the
    boundary conditions.
- **`:blocks`**, the same pieces grouped by kind:
  - **unknowns**: ``X_0, \dots, X_N``, then all the controls, then ``v``;
  - **constraints**: all the dynamics, then the path constraints at the nodes
    ``0, \dots, N``, then the boundary conditions.

```julia
model = beam(JuMPBackend(); grid_size=100, ordering=:blocks)
model[:ordering]    # :blocks
```

The NLP is the same in both orders, up to a permutation of the unknowns and of the
constraints. The unknowns keep their names (`x₁[3]`), and `model[:constraints]` lists the
constraints in CTDirect's order for both orderings, so the permutation can be computed
from the model.

!!! note "MOI groups the constraints by type"

    JuMP passes the constraints to the solver grouped by type: function (affine,
    quadratic, nonlinear) and set (equality, interval). The order of creation is kept only
    within a type. The dynamics, path constraints and boundary conditions of a problem are
    often of different types, so `:blocks` mostly changes the order of the unknowns.

## In the code

### A problem file declares data

A file of `ext/JuMPModels/` fills a `ProblemData` and hands it to the scheme; it writes no
discretization itself:

```julia
data = ProblemData(;
    state=[Component("x₁"; lower=x₁_l, upper=x₁_u, start=0.05), Component("x₂"; start=0.1)],
    control=[Component("u"; start=0.1)],
    costate=["∂x₁", "∂x₂"],
    times=v -> (t0, tf),
    dynamics=(x, u, v, t) -> [x[2], u[1]],
    lagrange=(x, u, v, t) -> u[1]^2,
    boundary=[(x₁_t0, (x0, xf, v) -> x0[1], x₁_t0), (x₁_tf, (x0, xf, v) -> xf[1], x₁_tf)],
)
return build(data, grid_size, scheme, ordering, args...; kwargs...)
```

| Field | Mathematical object |
| --- | --- |
| `state`, `control`, `variable` | the components, their bounds (box constraints) and start values (initial guess, a number or a vector over the nodes) |
| `times(v)` | ``(t_0, t_f)``, numbers or expressions of ``v`` |
| `dynamics(x, u, v, t)` | ``f``, at one node |
| `lagrange(x, u, v, t)` | ``f^0``, or `nothing` |
| `cost(I, x0, xf, v)` | the objective given the discretized integral ``I`` of ``f^0`` (default: ``I``), e.g. ``g`` alone or ``I + g`` |
| `path` | ``(c_l, c, c_u)`` |
| `boundary` | ``(b_l, b, b_u)`` |
| `state_expressions(x, v)` | what the dynamics constrain (default: ``x``), see below |

The functions receive the vectors of JuMP variables of one node and return JuMP
expressions: the scheme evaluates them at its stages (nodes or, for the midpoint rule,
``\frac{X_i + X_{i+1}}{2}``).

**Scaled models.** Some models are written on scaled variables but keep the dynamics of
the true quantities, for instance `space_shuttle` with ``h = 10^5\, \tilde h``. Writing
the scheme on ``\tilde h`` would divide those rows by ``10^5`` and change the NLP seen by
the solver. `state_expressions` returns the quantities the dynamics constrain, here
``(10^5 \tilde h, \dots)``, so that the scheme writes
``z(X_{i+1}) = z(X_i) + h \sum_j b_j f(Z_j)``.

### What the model stores

Besides the NLP, `build` stores in the JuMP model what is needed to read a
solution back:

| Key | Content | Used by |
| --- | --- | --- |
| `model[:x₁]`, … (one per state component) | the unknowns of the component, a container indexed by `0:N` | `state` |
| `model[:u]`, … (one per control component) | the unknowns of the component, indexed by `0:N` (trapezoidal rule) or `0:N-1` (one control per step, ``U_i`` on ``[t_i, t_{i+1}]``) | `control` |
| `model[:tf]`, … (one per variable component) | the unknown | `variable`, the time grid |
| `model[:∂x₁]`, … (one per costate name) | the dynamics constraints of the component, indexed by `1:N` | `costate` (their duals) |
| `model[:scheme]` | the scheme, `:trapeze`, `:euler`, `:euler_implicit` or `:midpoint` | `control`, `solution` |
| `model[:ordering]` | the order of the unknowns and constraints, `:interleaved` or `:blocks` | the comparisons with CTDirect |
| `model[:constraints]` | the constraints, in CTDirect's order whatever the ordering | the comparisons with CTDirect |
| `model[:time_grid]` | a function returning ``t_0, \dots, t_N``, evaluated after the solve when ``t_f`` is free | `time_grid` |
| `model[:state_components]`, `model[:control_components]`, `model[:variable_components]`, `model[:costate_components]` | the names above, in order | every accessor |

The accessors read these keys:

```julia
model = beam(JuMPBackend(); grid_size=100)
JuMP.set_optimizer(model, Ipopt.Optimizer)
JuMP.optimize!(model)

T = time_grid(model)          # t_0, …, t_N
x = state(model)              # function of t, interpolating X_0, …, X_N
u = control(model)            # function of t: linear (trapeze) or piecewise constant
p = costate(model)            # function of t, from the duals of the dynamics constraints
objective(model), iterations(model)
```

The costate is read from the duals of the ``N`` dynamics constraints of each component,
one per step: its sign and time conventions are not yet aligned with OptimalControl (see
[#326](https://github.com/control-toolbox/OptimalControlProblems.jl/issues/326)). When
the problem also exists in OptimalControl, `solution(model, pb)` builds a full
`CTModels.Solution` from these values, which can be plotted like an OptimalControl
solution.

With one control per step, the control is reported as CTDirect reports it: ``U_i`` on
``[t_i, t_{i+1})``, and ``U_{N-1}`` again at ``t_N``, so that it is given at the ``N+1``
nodes of the grid.
