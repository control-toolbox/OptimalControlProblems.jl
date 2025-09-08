The **particle steering problem** is a classical benchmark in time-optimal control.  
It models the task of steering a particle from a given initial state to a specified terminal state while minimising travel time.  
The system includes four state variables representing position and velocity components, and a single scalar control input that determines the steering direction.  
Control bounds and dynamics constraints must be satisfied throughout the trajectory [Athans & Falb 1966; Bryson & Ho 1975].

### Mathematical formulation

The problem can be stated as

```math
\begin{aligned}
\min_{x,u} \quad & J(x,u) = t_f \\[0.5em]
\text{s.t.} \quad &
\dot{x}_1(t) = x_3(t), \\[0.5em]
& \dot{x}_2(t) = x_4(t), \\[0.5em]
& \dot{x}_3(t) = a \cos(u(t)), \\[0.5em]
& \dot{x}_4(t) = a \sin(u(t)), \\[0.5em]
& u_{\min} \le u(t) \le u_{\max}, \\[0.5em]
& x(0) = x_s, \quad x(t_f) = x_f, \\[0.5em]
& t_f \ge 0.
\end{aligned}
```

where  

- the state $x = (x_1, x_2, x_3, x_4)$ are the particle states (position and velocity components),  
- the control $u$ is the steering control, bounded by $u_{\min} = -\pi/2$ and $u_{\max} = \pi/2$,  
- the parameter $a$ is a constant scaling the acceleration.

### Parameter values

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Acceleration magnitude | $a$ | 100 |
| Control lower bound | $u_{\min}$ | $-\pi/2$ |
| Control upper bound | $u_{\max}$ | $\pi/2$ |
| Initial state | $x_s$ | $(0,0,0,0)$ |
| Final state | $x_f$ | $(\text{unspecified},5,45,0)$ |

### Qualitative behaviour

The optimal trajectory typically exhibits **bang–bang control**, where the steering input switches between its extreme values to minimise time.  
The trajectory balances position and velocity to reach the terminal state in minimal time.  
Analytical solutions exist for simplified cases, while numerical methods are required for general conditions.

### Characteristics

- Four-dimensional state with single scalar control,  
- Time-optimal objective,  
- Control constraints and boundary conditions,  
- Bang–bang control structure with possible singular arcs,  
- Serves as a benchmark for time-optimal control methods.

### References

- Athans, M., & Falb, P.L. (1966). *Optimal Control: An Introduction to the Theory and Its Applications*. McGraw-Hill.  
  This classic text introduces the theory of optimal control, including time-optimal problems and bang–bang solutions, providing foundational concepts relevant to particle steering problems.

- Bryson, A.E., & Ho, Y.-C. (1975). *Applied Optimal Control: Optimization, Estimation, and Control*. Hemisphere Publishing.  
  The book presents methods and examples of time-optimal and energy-optimal control problems, including systems with bang–bang and singular arc solutions, directly relevant to the steering problem.

- More, J., Garbow, B., Hillstrom, K., & Watson, L. (2001). *COPS: Constrained Optimization Problem Set* (COPS3). Mathematics and Computer Science Division, Argonne National Laboratory. [https://www.mcs.anl.gov/~more/cops/cops3.pdf](https://www.mcs.anl.gov/~more/cops/cops3.pdf)  
  This report lists the particle steering problem as a benchmark for nonlinear constrained optimization, providing the original formulation and serving as a reference for numerical experimentation with optimal control solvers.

- PSOPT Example Set. *Particle Steering Problem*. [https://www.psopt.net/list-of-examples?utm_source=chatgpt.com](https://www.psopt.net/list-of-examples?utm_source=chatgpt.com)  
  A modern implementation of the particle steering problem for testing direct optimal control methods, illustrating numerical approaches for time-optimal trajectory problems.
