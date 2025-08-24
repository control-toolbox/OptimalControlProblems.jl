The *beam problem* is a classical benchmark in constrained optimal control. It originates from the Euler–Bernoulli beam model (see Bryson et al. 1963). The system describes the deflection of a clamped flexible beam under an external force. Both the state trajectory $x(\cdot)$ and the control $u(\cdot)$ are decision variables. The aim is to minimise the control effort subject to boundary conditions and a state inequality constraint on the displacement.

The problem can be written as

```math
\min_{x,\,u} J(x,u) = \int_0^1 u(t)^2 \, dt
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

and the constraints
```math
-10 \le u(t) \le 5, \qquad 
0 \le x_1(t) \le a,
```

where $a$ denotes the maximal admissible deflection (e.g. $a=0.1$ in the current implementation).

### Qualitative behaviour

- If $a \geq 1/4$, the inequality constraint is inactive and the solution is $x(t) = t(1-t)$.  
- If $a \in [1/6, 1/4]$, there is a single touch point at $t=1/2$.  
- If $a < 1/6$, a boundary arc appears, with the constraint active on an interval.  

These features illustrate the role of state inequality constraints in shaping optimal trajectories and controls.

### Characteristics

- Linear–quadratic dynamics and cost.  
- Asymmetric control bounds.  
- State inequality constraint induces touch points or boundary arcs depending on $a$.  
- Widely used as a benchmark in numerical optimal control libraries.

### References

- Bryson, A.E., Denham, W.F., & Dreyfus, S.E. (1963). *Optimal programming problems with inequality constraints I: necessary conditions for extremal solutions*. AIAA Journal.  
- BOCOP examples: Clamped Beam problem. [Examples-BOCOP.pdf](https://project.inria.fr/bocop/files/2017/05/Examples-BOCOP.pdf)
