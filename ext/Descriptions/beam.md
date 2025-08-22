The *beam problem* is a benchmark in constrained optimal control. It models the deflection of a clamped flexible beam under an external force. Both the state trajectory $x(\cdot)$ and the control $u(\cdot)$ are decision variables. The aim is to minimise the control effort while satisfying boundary conditions and the displacement constraint.

The problem is formulated as

```math
\min_{x,\,u} J(x,u) = \int_0^1 u(t)^2 \, dt
```

subject to the dynamics
```math
\dot{x}_1(t) = x_2(t), \qquad 
\dot{x}_2(t) = u(t)
```

with boundary conditions
```math
x_1(0) = 0, \quad x_2(0) = 1, \qquad 
x_1(1) = 0, \quad x_2(1) = -1
```

and the constraints
```math
-10 \le u(t) \le 5, \qquad 
0 \le x_1(t) \le 0.1
```

We emphasise that both the state x(t) and control u(t) are decision variables. The goal is to minimise the objective functional while satisfying all constraints.

### Characteristics

- Linear–quadratic dynamics and cost.  
- Asymmetric control bounds.  
- State inequality constraint may induce boundary arcs or active constraints.  
- Widely used as a benchmark in numerical optimal control libraries.

### References

- Bryson, A.E., Denham, W.F., & Dreyfus, S.E. (1963). *Optimal programming problems with inequality constraints I: necessary conditions for extremal solutions*. AIAA Journal.  
- BOCOP examples: Clamped Beam problem. [Examples-BOCOP.pdf](https://project.inria.fr/bocop/files/2017/05/Examples-BOCOP.pdf)
