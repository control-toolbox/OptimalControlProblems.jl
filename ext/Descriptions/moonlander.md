This problem models the optimal control of a lunar lander aiming to reach a target position on the lunar surface while minimizing fuel consumption. The system is described by a set of differential equations governing the motion of the lander, including its position, velocity, orientation, and angular velocity.

### Problem Formulation

The objective is to minimize the final time $t_f$ subject to the following constraints and dynamics:

- **State Variables**: Position ($p_1$, $p_2$), velocity ($dp_1$, $dp_2$), orientation ($\theta$), and angular velocity ($d\theta$).  
- **Control Inputs**: Thrust forces $F_1$ and $F_2$ applied along the lander's orientation.  
- **Final Time Constraint**: $0.1 \le t_f \le 1.0$.  
- **Control Constraints**: $0 \le F_1(t), F_2(t) \le \text{max\_thrust}$.  
- **Initial Conditions**: 

```math
p_1(0) = 0, \quad p_2(0) = 0, \quad dp_1(0) = 0, \quad dp_2(0) = 0, \quad \theta(0) = 0, \quad d\theta(0) = 0
```

- **Final Conditions**:

```math
p_1(t_f) = \text{target}[1], \quad p_2(t_f) = \text{target}[2], \quad dp_1(t_f) = 0, \quad dp_2(t_f) = 0
```

### System Dynamics

The dynamics of the lander are governed by

```math
\dot{p}_1 = dp_1
```

```math
\dot{p}_2 = dp_2
```

```math
\dot{dp}_1 = \frac{1}{m} F_{\rm tot}[1]
```

```math
\dot{dp}_2 = \frac{1}{m} F_{\rm tot}[2] - g
```

```math
\dot{\theta} = d\theta
```

```math
\dot{d\theta} = \frac{1}{I} \left( \frac{D}{2} (F_2 - F_1) \right)
```

where 

```math
F_{\rm tot} = 
\begin{bmatrix}
\cos\theta & -\sin\theta \\
\sin\theta & \cos\theta
\end{bmatrix}
\begin{bmatrix} 0 \\ F_1 + F_2 \end{bmatrix}_{1:2}
```

represents the total thrust vector in the inertial frame, $m$ is the mass of the lander, $I$ its moment of inertia, $D$ the distance between thrusters, and $g$ the lunar gravitational acceleration.

### Objective

The goal is to **minimize the final time** $t_f$:

```math
J = t_f \to \min
```

subject to the dynamics, boundary conditions, and control constraints.

### References

- Vanroye, L., Sathya, A., De Schutter, J., & Decré, W. (2023). FATROP: A Fast Constrained Optimal Control Problem Solver for Robot Trajectory Optimization and Control. *arXiv preprint arXiv:2303.16746*. Retrieved from https://arxiv.org/pdf/2303.16746
- Bryson, A. E., & Ho, Y.-C. (1975). *Applied Optimal Control: Optimization, Estimation, and Control*. Hemisphere Publishing.  
- Hull, D. G. (2007). *Fundamentals of Airplane Flight Mechanics* (Chapter on spacecraft and landing). Springer.  
