The **Van der Pol oscillator control problem** is a benchmark in nonlinear optimal control.  
It models a two-dimensional oscillator with nonlinear damping and a control input.  
The state vector is $x(t) = [x_1(t), x_2(t)]^\top$, and the control variable is $u(t)$.  
The goal is to minimise a quadratic cost functional over a **fixed horizon** $T = 2$ while steering the system from a given initial condition.

### Mathematical formulation

The problem can be stated as

```math
\begin{aligned}
\min_{x,u} \quad & J(x,u) = \frac{1}{2} \int_0^2 \big( x_1(t)^2 + x_2(t)^2 + u(t)^2 \big) \, dt \\[0.5em]
\text{s.t.} \quad &
\dot{x}_1(t) = x_2(t), \\[0.5em]
& \dot{x}_2(t) = \varepsilon \, \omega \, (1 - x_1(t)^2) x_2(t) - \omega^2 x_1(t) + u(t), \\[0.5em]
& x(0) = [1, 0], \\[0.5em]
& u(t) \in \mathbb{R}.
\end{aligned}
```

### Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Oscillator frequency | $\omega$ | 1 |
| Nonlinearity coefficient | $\varepsilon$ | 1 |
| Final time | $T$ | 2 |

### Qualitative behaviour

- The optimal control $u(t)$ regulates the nonlinear oscillations to minimise both state excursions and control effort.  
- Due to the quadratic cost, the control is typically **smooth**, without bang–bang or singular arcs.  
- Serves as a standard test for **direct transcription methods** and numerical solvers for nonlinear optimal control problems.

### Characteristics

- Two-dimensional nonlinear dynamics with cubic damping,  
- Quadratic cost on both states and control,  
- Fixed final time,  
- Unconstrained control (extensions may include bounds),  
- Useful benchmark for testing optimal control algorithms and integration schemes.

### References

- **Dymos Documentation**. [*The Van der Pol Oscillator*.](https://openmdao.github.io/dymos/examples/vanderpol/vanderpol.html).
  Provides an implementation of the Van der Pol oscillator in an optimal control framework, illustrating problem setup, discretisation, and solution.

- **mintOC**. [*Van der Pol Oscillator*](https://mintoc.de/index.php/Van_der_Pol_Oscillator).
  Presents the Van der Pol optimal control problem with numerical results, highlighting cost function and control profiles.

- James, E.M. (1974). *Time Optimal Control and the Van der Pol Oscillator*. *IMA Journal of Applied Mathematics*, 13(1), 67–78.  
  Discusses theoretical aspects of time-optimal control for the Van der Pol oscillator, including analysis of control strategies.

- Van Dooren, R. (1987). *Numerical Study of the Controlled Van der Pol Oscillator in Optimal Control*. *Numerische Mathematik*, 51(4), 471–485.  
  Explores numerical methods for solving the Van der Pol optimal control problem, providing insights into solution behavior.