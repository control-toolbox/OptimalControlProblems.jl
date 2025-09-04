The **clamped beam problem** is a classical benchmark in constrained optimal control.  
It models the deflection of a flexible beam that is fixed (clamped) at both ends and subject to an external force.  
Originating from the Euler–Bernoulli beam model [Bryson et al. 1963](https://doi.org/10.2514/3.2107), it is widely used in benchmark suites.  
Both the state trajectory $x(\cdot)$ and the control $u(\cdot)$ are decision variables, and the aim is to minimise the control effort under boundary conditions and a state inequality constraint on the beam deflection.

### Mathematical formulation

The problem can be stated as

```math
\begin{aligned}
\min_{x,u} \quad & J(x,u) = \int_0^1 u^2(t) \,\mathrm{d}t \\[1em]
\text{s.t.} \quad & \dot{x}_1(t) = x_2(t), \quad \dot{x}_2(t) = u(t), \\[0.5em]
& x_1(0) = 0, \; x_1(1) = 0, \; x_2(0) = 1, \; x_2(1) = -1, \\[0.5em]
& 0 \le x_1(t) \le a.
\end{aligned}
```

where $a$ denotes the maximal admissible deflection (e.g. $a=0.1$ in the current implementation).

### Qualitative behaviour

This problem involves a **second-order state constraint**.  
Differentiating the inequality twice reveals that the control is directly related to the curvature of the state:

```math
\dot{x}_1(t) = x_2(t), \quad \ddot{x}_1(t) = u(t).
```

Along a **boundary arc**, where $x_1$ remains constant, both derivatives vanish.  
Consequently, the control along such an arc is zero:

```math
u(t) = 0.
```

In optimal control terminology, a **touch point** refers to the case where the state hits the bound at a single instant,  
while a **boundary arc** designates an interval where the state remains on the bound.

The qualitative behaviour of the optimal trajectory depends on the parameter $a$:

- **Case 1: $a \ge 1/4$**.  
  The inequality constraint is **inactive**. The optimal solution is simply the unconstrained trajectory: $x(t) = t\, (1-t)$.

- **Case 2: $1/6 \le a \le 1/4$**.  
  The trajectory touches the boundary exactly once (a **touch point**) at $t=1/2$.  
  There is no interval of constraint activation, but the single contact reflects the influence of the state bound.

- **Case 3: $a < 1/6$**.  
  A **boundary arc** appears: the state constraint is active over a finite interval.  
  Along this interval, the control vanishes: $u(t) = 0$.

These cases illustrate how second-order state inequality constraints can yield unconstrained solutions, isolated touch points, or extended boundary arcs, thereby shaping both the trajectory and the control.

### Characteristics

- Linear–quadratic dynamics and cost.  
- State inequality constraint induces either unconstrained motion, touch points, or boundary arcs depending on the parameter $a$.  
- Serves as a benchmark for optimal control methods handling state constraints.

### References

- Bryson, A.E., Denham, W.F., & Dreyfus, S.E. (1963). *Optimal programming problems with inequality constraints I: necessary conditions for extremal solutions*.  
  AIAA Journal. [https://doi.org/10.2514/3.2107](https://doi.org/10.2514/3.2107)  
  This seminal work develops the theoretical foundation for optimal control problems with inequality constraints, including the derivation of necessary conditions for extremal solutions. It provides the basis for analyzing state-constrained control problems like the clamped beam.

- BOCOP examples: Clamped Beam problem. [https://project.inria.fr/bocop/files/2017/05/Examples-BOCOP.pdf](https://project.inria.fr/bocop/files/2017/05/Examples-BOCOP.pdf)  
  This example illustrates the practical implementation of the clamped beam problem in BOCOP, providing a benchmark for testing optimal control methods under state inequality constraints.