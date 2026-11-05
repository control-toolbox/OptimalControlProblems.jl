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

!!! note "Availability in this package"

    The JuMP models implement the **trapezoidal rule**. The three other schemes are
    described here as the reference conventions of CTDirect; they come to the JuMP
    models with [#369](https://github.com/control-toolbox/OptimalControlProblems.jl/issues/369).

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

The JuMP models follow CTDirect's **interleaved** order, which matters for the sparsity
pattern seen by the linear solver:

- **unknowns**: node by node, the state then the control,
  ``X_0, U_0, X_1, U_1, \dots, X_N, U_N``, then ``v``;
- **constraints**: for each step ``i = 0, \dots, N-1``, the dynamics of the step then the
  path constraints at node ``i``; then the path constraints at node ``N``; then the
  boundary conditions.

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
return build_trapeze(data, grid_size, args...; kwargs...)
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
expressions: the scheme evaluates them at the nodes it needs.

**Scaled models.** Some models are written on scaled variables but keep the dynamics of
the true quantities, for instance `space_shuttle` with ``h = 10^5\, \tilde h``. Writing
the scheme on ``\tilde h`` would divide those rows by ``10^5`` and change the NLP seen by
the solver. `state_expressions` returns the quantities the dynamics constrain, here
``(10^5 \tilde h, \dots)``, so that the scheme writes
``z(X_{i+1}) = z(X_i) + \frac{h}{2}(f_i + f_{i+1})``.

### What the model stores

Besides the NLP, `build_trapeze` stores in the JuMP model what is needed to read a
solution back:

| Key | Content | Used by |
| --- | --- | --- |
| `model[:x₁]`, … (one per state and control component) | the unknowns of the component, a container indexed by `0:N` | `state`, `control` |
| `model[:tf]`, … (one per variable component) | the unknown | `variable`, the time grid |
| `model[:∂x₁]`, … (one per costate name) | the dynamics constraints of the component, indexed by `1:N` | `costate` (their duals) |
| `model[:time_grid]` | a function returning ``t_0, \dots, t_N``, evaluated after the solve when ``t_f`` is free | `time_grid` |
| `model[:state_components]`, `model[:control_components]`, `model[:variable_components]`, `model[:costate_components]` | the names above, in order | every accessor |

The accessors read these keys:

```julia
model = beam(JuMPBackend(); grid_size=100)
JuMP.set_optimizer(model, Ipopt.Optimizer)
JuMP.optimize!(model)

T = time_grid(model)          # t_0, …, t_N
x = state(model)              # function of t, interpolating X_0, …, X_N
u = control(model)            # function of t, interpolating the controls
p = costate(model)            # function of t, from the duals of the dynamics constraints
objective(model), iterations(model)
```

The costate is read from the duals of the ``N`` dynamics constraints of each component,
one per step: its sign and time conventions are not yet aligned with OptimalControl (see
[#326](https://github.com/control-toolbox/OptimalControlProblems.jl/issues/326)). When
the problem also exists in OptimalControl, `solution(model, pb)` builds a full
`CTModels.Solution` from these values, which can be plotted like an OptimalControl
solution.
