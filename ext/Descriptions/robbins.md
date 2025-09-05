This problem was introduced by Robbins (1980) as a benchmark involving a **third-order state constraint**:

```math
x_1^{(3)}(t) = u(t), \qquad x_1(t) \ge 0,
```

with control $u(t)$ and cost functional

```math
J(x,u) = \int_0^T \big( \alpha x_1(t) + \beta x_1(t)^2 + \gamma u(t)^2 \big) \, dt.
```

The system is represented in first-order form as

```math
\begin{aligned}
\dot x_1(t) &= x_2(t), \\
\dot x_2(t) &= x_3(t), \\
\dot x_3(t) &= u(t),
\end{aligned}
```

with boundary conditions

```math
x(0) = (1, -2, 0), \qquad x(T) = (0,0,0),
```

and horizon $T=10$. In our simulations we use parameters $\alpha=3$, $\beta=0$, $\gamma=0.5$.

### Qualitative behaviour

Robbins (1980) showed that the exact solution has **infinitely many isolated contact points**, where the state constraint $x_1(t)=0$ is active.  
The unconstrained arcs between contacts decrease geometrically, leading to an **accumulation point**, followed by the trivial singular arc $u=0$, $x=0$.  

Numerically, capturing all contact points is challenging because the unconstrained arcs rapidly shrink below the resolution of a given discretisation.  
Simulations reproduce the initial part of the solution, showing the first few contact points.

### References

- Robbins, H. M. (1980). *Junction phenomena for optimal control with state-variable inequality constraints of third order*. Journal of Optimization Theory and Applications, 31, 85–99.  
- Hermant, A. (2008). *Sur l'algorithme de tir pour les problèmes de commande optimale avec contraintes sur l'état* (PhD thesis, École Polytechnique X).  
- Jacobson, D. H., Lele, M. M., & Speyer, J. L. (1971). *New necessary conditions of optimality for control problems with state-variable inequality constraints*. Journal of Mathematical Analysis and Applications, 35, 255–284.  
- BOCOP repository: https://github.com/control-toolbox/bocop
