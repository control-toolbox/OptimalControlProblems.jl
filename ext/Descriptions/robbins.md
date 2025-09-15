The **Robbins benchmark problem** is a classical optimal control problem introduced by Robbins (1980), involving a **third-order state constraint**:

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

### Parameter values

- Weight on state $x_1$: $\alpha = 3$  
- Weight on squared state: $\beta = 0$  
- Weight on squared control: $\gamma = 0.5$  
- Final time: $T = 10$  

### Qualitative behaviour

The exact solution exhibits **infinitely many isolated contact points**, where the state constraint $x_1(t)=0$ is active.  
Between these contact points, the unconstrained arcs decrease geometrically, accumulating at a point, followed by a trivial singular arc where $u=0$ and $x=0$.  

Numerically, capturing all contact points is challenging because the unconstrained arcs quickly shrink below the resolution of a given discretisation.  
Simulations reproduce the initial portion of the solution, showing the first few contact points.  
The control $u(t)$ typically exhibits a **bang–bang structure** with possible singular arcs, while $x_2(t)$ and $x_3(t)$ evolve to satisfy the dynamics.

### Characteristics

- Third-order state constraint with inequality $x_1(t) \ge 0$,  
- Linear dynamics in first-order representation,  
- Bang–bang control and singular arcs with infinitely many contact points,  
- Serves as a benchmark for direct transcription and NLP solvers handling state-constrained optimal control problems.

### References

- Robbins, H. M. (1980). *Junction phenomena for optimal control with state-variable inequality constraints of third order*. Journal of Optimization Theory and Applications, 31, 85–99.  
  This is the original paper introducing the Robbins problem. It formulates the third-order state-constrained optimal control problem, describes the accumulation of contact points, and provides theoretical analysis of the junction phenomena.

- Hermant, A. (2008). *Sur l'algorithme de tir pour les problèmes de commande optimale avec contraintes sur l'état* (PhD thesis, École Polytechnique X).  
  This thesis discusses numerical shooting methods for state-constrained optimal control problems, including the Robbins problem. It provides practical insights into solving problems with multiple contact points and complex singular arcs.

- Jacobson, D. H., Lele, M. M., & Speyer, J. L. (1971). *New necessary conditions of optimality for control problems with state-variable inequality constraints*. Journal of Mathematical Analysis and Applications, 35, 255–284.  
  This foundational work introduces necessary conditions for optimality in control problems with state-variable inequality constraints, forming the theoretical basis for analyzing and solving problems like Robbins.

- BOCOP repository: [Robbins problem](https://github.com/control-toolbox/bocop)  
  Contains a practical implementation of the Robbins benchmark in the BOCOP optimal control framework. This resource allows testing and benchmarking direct transcription and NLP solvers on the Robbins problem, reproducing the first few contact points in numerical simulations.
