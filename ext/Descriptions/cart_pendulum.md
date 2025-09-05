The **cart-pole swing-up problem** is a classical benchmark in underactuated optimal control.  
It consists of swinging a pendulum mounted on a cart from its downward equilibrium to the upright position while controlling the horizontal motion of the cart.  
The objective is to reach the upright position in **minimum time**, subject to cart and pendulum constraints.  
This problem is widely used to test trajectory optimisation and control algorithms for underactuated systems.

### Mathematical formulation

The problem can be stated as

```math
\begin{aligned}
\min_{x,v,\theta,\omega,F_{\rm ex},T} \quad & T \\[0.5em]
\text{s.t.} \quad &
\dot{x} = v, \quad \dot{v} = -\frac{1}{J} \, c, \quad \dot{\theta} = \omega, \quad \dot{\omega} = \alpha(\dot{v}), \\[0.5em]
& x(0) = 0, \quad \theta(0) = 0, \quad \omega(0) = 0, \\[0.25em]
& \theta(T) = \pi, \quad \omega(T) = 0, \\[0.5em]
& |x(t)| \le 1, \quad |v(t)| \le 2, \quad |F_{\rm ex}(t)| \le 5, \quad T \ge 0.1.
\end{aligned}
```

The auxiliary functions defining the dynamics are

```math
\begin{aligned}
\alpha(\ddot{x}) &= \frac{0.5 \, L \, m}{I + 0.25 \, m L^2} \big(-\ddot{x} \cos\theta - g \sin\theta\big), \\[0.5em]
\text{ddCOG} &= L \, \omega \, [-\sin\theta, \cos\theta] + \frac{L}{2} [\cos\theta, \sin\theta] \, \alpha(\ddot{x}) + [\ddot{x},0], \\[0.5em]
\text{FXFY} &= m \, \text{ddCOG} + [0, m g], \\[0.5em]
c &= -\text{FXFY}_x + F_{\rm ex} - m_{\rm cart} \ddot{x} - J \ddot{x}.
\end{aligned}
```

These represent the intermediate computations:

- The function $\alpha(\ddot{x})$ computes the pendulum’s angular acceleration due to cart acceleration and gravity.  
- The variable $\text{ddCOG}$ represents the acceleration of the pendulum’s centre of gravity.  
- The variable $\text{FXFY}$ is the net force vector acting on the pendulum.  
- The variable $c$ combines pendulum and cart dynamics to determine the net horizontal force for the system.  

The system parameters are:

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Pendulum mass | $m$ | 1 kg |
| Cart mass | $m_{\rm cart}$ | 0.5 kg |
| Pendulum length | $L$ | 1 m |
| Pendulum inertia | $I$ | 0.0833 kg·m² |
| Gravity | $g$ | 9.81 m/s² |
| Effective cart mass | $J$ | 0.5 kg |
| Maximum force | $F_{\rm max}$ | 5 N |
| Maximum cart position | $x_{\rm max}$ | 1 m |
| Maximum cart velocity | $v_{\rm max}$ | 2 m/s |

### Qualitative behaviour

The optimal control trajectory typically follows a **bang–singular–bang** pattern:

- Maximum force is initially applied to accelerate the pendulum and cart (bang arc).  
- A singular arc follows, balancing the pendulum energy to reach the upright position.  
- Maximum force is applied again to stabilise and position the cart.

The pendulum angle evolves from downward ($\theta = 0$) to upright ($\theta = \pi$), while the cart remains within its bounds.  
Underactuation makes the problem challenging because the pendulum cannot be actuated directly and the control must exploit coupled dynamics.

### Characteristics

- Nonlinear, underactuated dynamics.  
- Minimum-time objective with state and control constraints.  
- Serves as a benchmark for trajectory optimisation and control of underactuated systems.

### References

- **Åström, K. J., & Furuta, K. (2000).** *Swinging up a pendulum by energy control*. Automatica, 36(2), 287–295.  
  [doi.org/10.1016/S0005-1098(99)00140-5](https://doi.org/10.1016/S0005-1098(99)00140-5)  
  This seminal paper introduces energy-based control strategies for swinging up an inverted pendulum, emphasising the critical role of the pivot's acceleration relative to gravity. It provides foundational insights into the challenges of controlling underactuated systems.

- **Vanroye, L., Sathya, A., De Schutter, J., & Decré, W. (2023).** *FATROP: A Fast Constrained Optimal Control Problem Solver for Robot Trajectory Optimisation and Control*. arXiv preprint arXiv:2303.16746.  
  [arxiv.org/abs/2303.16746](https://arxiv.org/abs/2303.16746)  
  This paper presents FATROP, a solver designed to efficiently handle constrained optimal control problems, including the cart-pole swing-up. It demonstrates how modern numerical methods can compute minimum-time trajectories while respecting state and control constraints, making it highly relevant for implementing the problem in practice.

- **[Source code files for the Cart-Pendulum problem](https://gitlab.kuleuven.be/robotgenskill/fatrop/fatrop_benchmarks/-/tree/main/cart_pendulum).**  
  This repository contains the implementation of the cart-pendulum problem used in the FATROP benchmarks, providing practical examples of state, control, and dynamics formulations.

- **Tedrake, R. (2024).** *Underactuated Robotics: Algorithms for Walking, Running, Swimming, Flying, and Manipulation (Course Notes for MIT 6.832)*.  
  [underactuated.mit.edu/](https://underactuated.mit.edu/)  
  This comprehensive resource offers detailed discussions on the cart-pole system, including its dynamics, control strategies, and applications in robotics. It serves as an excellent reference for understanding the theoretical and practical aspects of underactuated systems.
