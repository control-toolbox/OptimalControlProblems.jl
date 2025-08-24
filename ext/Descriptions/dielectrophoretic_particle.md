The *dielectrophoretic particle problem* is a classical time-optimal control benchmark for microfluidic particle manipulation. It models the motion of a particle under a dielectrophoretic force, where the control voltage applied to electrodes directly influences the particle trajectory. Both the particle position and an auxiliary state related to its dipole moment, as well as the control voltage, are decision variables. The objective is to transfer the particle from an initial position to a target position in minimal time, while satisfying bounds on the control input and maintaining the auxiliary state dynamics.

The problem can be written as

```math
\min_{x,\,y,\,u,\,t_f} J = t_f
```

subject to the dynamics
```math
\dot{x}(t) = y(t) u(t) + \alpha u(t)^2, \qquad 
\dot{y}(t) = -c y(t) + u(t),
```

with boundary conditions
```math
x(0) = x_0, \quad y(0) = 0, \qquad 
x(t_f) = x_f,
```

and the constraints
```math
-1 \le u(t) \le 1, \qquad 
t_f \ge 0,
```

where $x_0$ and $x_f$ are the initial and final particle positions, $\alpha$ is a coefficient representing nonlinear interaction with the field, and $c$ is a damping coefficient.

### Qualitative behaviour

- The optimal control often saturates at the bounds $u = \pm 1$, characteristic of time-optimal problems.  
- The auxiliary state $y(t)$ evolves according to both the control and its own decay, influencing the particle's acceleration nonlinearly.  
- The final time $t_f$ is free and adjusted by the optimisation to achieve minimal transfer time.

### Characteristics

- Nonlinear dynamics with bilinear and quadratic terms in the control.  
- Time-optimal objective with control bounds.  
- Free end-time formulation.  
- Widely used as a benchmark in numerical optimal control for microfluidic applications.

### References

- Chang, D. E., Petit, N., & Rouchon, P. (2005). *Time-optimal control of a particle in a dielectrophoretic system*. International Journal of Robust and Nonlinear Control, 15(7), 769–784.  
- IFAC 2005 examples: Dielectrophoretic particle control. [PDF](https://cas.minesparis.psl.eu/~petit/papers/ifac2005/ifac05dec.pdf)
