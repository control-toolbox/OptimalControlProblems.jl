The **robot arm motion problem** is a benchmark in time-optimal control.  
It models a robotic arm moving between two points in space, represented as a rigid bar of total length $L$ pivoting at the origin of a spherical coordinate system.  
The system includes six state variables: the radial position $\rho(t)$ and its velocity $d\rho(t)$, the horizontal angle $\theta(t)$ and its angular velocity $d\theta(t)$, and the vertical angle $\phi(t)$ and its angular velocity $d\phi(t)$.  
The control variables are $u_\rho(t)$, $u_\theta(t)$, and $u_\phi(t)$, representing the forces or torques applied along the radial and angular directions.  
The objective is to **minimise the total transfer time** while satisfying state and control constraints [BOCOP Robot Arm Example](https://github.com/control-toolbox/bocop/tree/main/bocop).

### Mathematical formulation

The problem can be stated as

```math
\begin{aligned}
\min_{\rho, d\rho, \theta, d\theta, \phi, d\phi, u_\rho, u_\theta, u_\phi, T} \quad & J = T \\[0.5em]
\text{s.t.} \quad &
\dot{\rho}(t) = d\rho(t), \quad \dot{\theta}(t) = d\theta(t), \quad \dot{\phi}(t) = d\phi(t), \\[0.5em]
& \dot{d\rho}(t) = \frac{u_\rho(t)}{L}, \quad
\dot{d\theta}(t) = \frac{3 u_\theta(t)}{I_\theta(t)}, \quad
\dot{d\phi}(t) = \frac{3 u_\phi(t)}{I_\phi(t)}, \\[0.5em]
& I_\theta(t) = ((L-\rho(t))^3 + \rho(t)^3) \sin^2(\phi(t)), \quad
I_\phi(t) = (L-\rho(t))^3 + \rho(t)^3, \\[0.5em]
& 0 \le \rho(t) \le L, \quad -\pi \le \theta(t) \le \pi, \quad 0 \le \phi(t) \le \pi, \\[0.5em]
& |u_\rho(t)| \le 1, \quad |u_\theta(t)| \le 1, \quad |u_\phi(t)| \le 1, \\[0.5em]
& \rho(0) = \rho(T) = 4.5, \quad \theta(0) = 0, \quad \theta(T) = \frac{2\pi}{3}, \quad \phi(0) = \phi(T) = \frac{\pi}{4}, \\[0.5em]
& d\rho(0) = d\rho(T) = 0, \quad d\theta(0) = d\theta(T) = 0, \quad d\phi(0) = d\phi(T) = 0, \\[0.5em]
& T \ge 0.1.
\end{aligned}
```

The horizon is **free**, as the total transfer time $T$ is optimised.

### Parameter values

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Total arm length | $L$ | 5 |
| Initial radial position | $\rho_0$ | 4.5 |
| Initial vertical angle | $\phi_0$ | $\pi/4$ |
| Final horizontal angle | $\theta_f$ | $2\pi/3$ |
| Maximum radial control | $u_\rho^{\max}$ | 1 |
| Maximum horizontal control | $u_\theta^{\max}$ | 1 |
| Maximum vertical control | $u_\phi^{\max}$ | 1 |
| Minimum final time | $T_{\min}$ | 0.1 |

### Qualitative behaviour

The optimal solution exploits the **degrees of freedom of the spherical coordinate system** to minimise the motion time.  
Control inputs $u_\rho$, $u_\theta$, and $u_\phi$ typically follow **bang–bang profiles**, alternating between maximum and minimum values to reach the target efficiently.  
The dynamics are nonlinear due to the dependence of the moments of inertia $I_\theta$ and $I_\phi$ on the radial and angular positions, which affects the acceleration of the arm.

### Characteristics

- Nonlinear, second-order dynamics in spherical coordinates,  
- Free final time with state and control constraints,  
- Bang–bang control structure in the time-optimal solution,  
- Serves as a benchmark for direct transcription and NLP solvers in robotic motion planning.

### References

- Mössner-Beigel, M. (1989). *Thesis*, Heidelberg University.  
  Introduces the time-optimal robotic arm problem and its theoretical formulation.  

- Vanderbei, R. J. (2001). *Case studies in trajectory optimization: Trains, Planes, and Other Pastimes*.  
  Provides numerical strategies for robotic motion planning and discusses control substitution techniques for time-optimal trajectories.  

- BOCOP examples: Robot arm problem. [BOCOP Robot Arm Example](https://github.com/control-toolbox/bocop/tree/main/bocop)  
  Demonstrates the practical implementation using direct transcription and NLP solvers for robotic motion problems.
