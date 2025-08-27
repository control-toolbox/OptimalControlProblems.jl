This problem models the **minimum-time maneuvering of a quadrotor** from an initial position to a target position while respecting thrust and tilt constraints.  
The quadrotor is modelled as a rigid body with translational and rotational dynamics, subject to actuator limits and tilt angle restrictions.

### System Description

The system has **8 states** and **4 controls**:

- **States**: 
  - $p_1, p_2, p_3$: position coordinates of the quadrotor  
  - $v_1, v_2, v_3$: linear velocities  
  - $\phi$: roll angle  
  - $\theta$: pitch angle  

- **Controls**: 
  - $a_t$: total thrust magnitude  
  - $\dot{\phi}$: roll rate  
  - $\dot{\theta}$: pitch rate  
  - $\psi$: yaw angle

### Constraints

- **Time horizon**: $t_f \ge 0.1$  
- **State constraints**: 

```math
-\pi/2 \le \phi(t), \theta(t) \le \pi/2
```

- **Control constraints**: 

```math
a_{\min} \le a_t(t) \le a_{\max}, \quad
-d_{\text{tilt,max}} \le \dot{\phi}(t), \dot{\theta}(t) \le d_{\text{tilt,max}}
```

- **Tilt path constraint**: 

```math
\cos\theta(t) \cos\phi(t) \ge \cos(\text{tiltmax})
```

- **Boundary conditions**:  

```math
p(0) = p_0, \quad v(0) = v_0, \quad \phi(0) = \phi_0, \quad \theta(0) = \theta_0
```

```math
p(t_f) = p_f, \quad v(t_f) = v_f
```

### Dynamics

The quadrotor dynamics are described by

```math
\dot{p} = v, \quad \dot{v} = a
```

```math
\dot{\phi} = \dot{\phi}_{\rm control}, \quad \dot{\theta} = \dot{\theta}_{\rm control}
```

where the acceleration $a$ is computed from the total thrust $a_t$ and orientation angles $(\phi, \theta, \psi)$:

```math
R = 
\begin{bmatrix}
c_\psi c_\theta & c_\psi s_\theta s_\phi - s_\psi c_\phi & c_\psi s_\theta c_\phi + s_\psi s_\phi \\
s_\psi c_\theta & s_\psi s_\theta s_\phi + c_\psi c_\phi & s_\psi s_\theta c_\phi - c_\psi s_\phi \\
-s_\theta & c_\theta s_\phi & c_\theta c_\phi
\end{bmatrix}, \quad
a = R \begin{bmatrix} 0 \\ 0 \\ a_t \end{bmatrix} + \begin{bmatrix} 0 \\ 0 \\ -g \end{bmatrix}
```

with $c_\phi = \cos\phi$, $s_\phi = \sin\phi$, etc.

### Objective

The goal is to **minimize the final time** $t_f$, with a small regularization on control inputs and yaw angle:

```math
J = t_f + \int_0^{t_f} \Big( 1e-8 (a_t^2 + \phi^2 + \theta^2 + \psi^2) + 1e2 (\psi - \psi_0)^2 \Big) dt \to \min
```

### References

- Vanroye, L., Sathya, A., De Schutter, J., & Decré, W. (2023). FATROP: A Fast Constrained Optimal Control Problem Solver for Robot Trajectory Optimization and Control. *arXiv preprint arXiv:2303.16746*. Retrieved from https://arxiv.org/pdf/2303.16746
- Mellinger, D., & Kumar, V. (2011). Minimum snap trajectory generation and control for quadrotors. *2011 IEEE International Conference on Robotics and Automation*, 2520–2525.  
- Faessler, M., Franchi, A., & Scaramuzza, D. (2018). Differential flatness of quadrotor dynamics subject to rotor drag for accurate tracking of high-speed trajectories. *IEEE Robotics and Automation Letters*, 3(2), 620–626.  
