The *(clamped) beam problem* is a classical benchmark in constrained optimal control. It originates from the Euler–Bernoulli beam model (see Bryson et al. 1963). The system describes the deflection of a clamped flexible beam under an external force. Both the state trajectory $x(\cdot)$ and the control $u(\cdot)$ are decision variables. The aim is to minimise the control effort subject to boundary conditions and a state inequality constraint on the displacement.

### Mathematical formulation

The problem can be written as

```math
\min_{x,\,u} J(x,u) = \int_0^1 u^2(t) \, \mathrm{d}t
```

subject to the dynamics
```math
\dot{x}_1(t) = x_2(t), \qquad 
\dot{x}_2(t) = u(t),
```

with boundary conditions
```math
x_1(0) = 0, \quad x_1(1) = 0, \qquad 
x_2(0) = 1, \quad x_2(1) = -1,
```

and the state constraint
```math
0 \le x_1(t) \le a,
```

where $a$ denotes the maximal admissible deflection (e.g. $a=0.1$ in the current implementation).

### Qualitative behaviour

This problem features a second-order state constraint. In particular, the control can be recovered by differentiating the constraint twice:

```math
\dot{x}_1(t) = x_2(t), \quad \ddot{x}_1(t) = u(t)
```

Along a **boundary arc**, where $x_1$ remains constant, both the first and second derivatives of $x_1$ vanish. As a result, the control along such an arc is zero:

```math
u(t) = 0
```

The qualitative behaviour of the optimal trajectory depends on the value of the parameter $a$:

- **Case 1: $a \ge 1/4$**. In this regime, the inequality constraint is **inactive**, meaning it does not influence the trajectory. The optimal solution is simply

```math
x(t) = t(1-t)
```

which is the unconstrained trajectory.

- **Case 2: $1/6 \le a \le 1/4$**. Here, the trajectory **touches the boundary exactly once**, at $t = 1/2$. There is no interval where the constraint is active, but the single touch reflects the influence of the state inequality.

- **Case 3: $a < 1/6$**. For smaller values of $a$, a **boundary arc appears**, meaning the state constraint becomes active over a finite interval. Along this interval, the control is zero:

```math
u(t) = 0.
```

These three cases clearly illustrate how **state inequality constraints** shape both the form of the optimal trajectories and the corresponding control actions. In particular, they show that second-order state constraints can either have no effect, a single-touch effect, or influence a whole interval of the trajectory.

### Characteristics

- Linear–quadratic dynamics and cost.  
- Asymmetric control bounds.  
- State inequality constraint induces touch points or boundary arcs depending on $a$.  
- Widely used as a benchmark in numerical optimal control libraries.

### References

- Bryson, A.E., Denham, W.F., & Dreyfus, S.E. (1963). *Optimal programming problems with inequality constraints I: necessary conditions for extremal solutions*. AIAA Journal.  
- BOCOP examples: Clamped Beam problem. [Examples-BOCOP.pdf](https://project.inria.fr/bocop/files/2017/05/Examples-BOCOP.pdf).
