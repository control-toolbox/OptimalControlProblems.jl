This problem models the **minimum-time maneuvering of a truck with two trailers** while respecting steering, velocity, and articulation constraints.  
The goal is to move the vehicle from an initial configuration to a target position and orientation while minimizing final time and reducing excessive trailer articulation.

### System Description

The system has **7 states** and **2 controls**:

- **States**:  
  - $x_2, y_2$: position of the rear trailer axle  
  - $\theta_0, \theta_1, \theta_2$: orientations of the truck and two trailers  
  - $v_0$: longitudinal velocity of the truck  
  - $\delta_0$: steering angle of the truck

- **Controls**:  
  - $\dot{v}_0$: acceleration of the truck  
  - $\dot{\delta}_0$: steering rate

- **Auxiliary variables**:  
  - $\beta_{01} = \theta_0 - \theta_1$  
  - $\beta_{12} = \theta_1 - \theta_2$  

### Constraints

- **Time horizon**: $1 \le t_f \le 1000$  
- **State constraints**:

```math
-\pi/2 \le \theta_0(t), \theta_1(t) \le \pi/2, \quad
-\frac{0.2 v_{\rm max}}{1} \le v_0(t) \le \frac{0.2 v_{\rm max}}{1}, \quad
-\pi/6 \le \delta_0(t) \le \pi/6
```

- **Control constraints**:

```math
-1 \le \dot{v}_0(t) \le 1, \quad
-\pi/10 \le \dot{\delta}_0(t) \le \pi/10
```

- **Path constraints**:

```math
-\pi/2 \le \beta_{01}(t), \beta_{12}(t) \le \pi/2
```

- **Boundary conditions**:  

```math
x_2(0) = x_{2,0}, \quad y_2(0) = y_{2,0}, \quad \theta_0(0) = \theta_{0,0}, \quad \theta_1(0) = \theta_{1,0}, \quad \theta_2(0) = \theta_{2,0}
```

```math
x_2(t_f) = x_{2,f}, \quad y_2(t_f) = y_{2,f}, \quad \theta_2(t_f) = \theta_{2,f}, \quad \beta_{01}(t_f) = \theta_{0,f} - \theta_{1,f}, \quad \beta_{12}(t_f) = \theta_{1,f} - \theta_{2,f}
```

### Dynamics

The truck-trailer kinematics are governed by

```math
\begin{aligned}
\dot{\theta}_0 &= \frac{v_0}{L_0} \tan\delta_0, \\
\dot{\theta}_1 &= \frac{v_0}{L_1} \sin\beta_{01} - \frac{M_0}{L_1} \cos\beta_{01} \, \dot{\theta}_0, \\
v_1 &= v_0 \cos\beta_{01} + M_0 \sin\beta_{01} \, \dot{\theta}_0, \\
\dot{\theta}_2 &= \frac{v_1}{L_2} \sin\beta_{12} - \frac{M_1}{L_2} \cos\beta_{12} \, \dot{\theta}_1, \\
v_2 &= v_1 \cos\beta_{12} + M_1 \sin\beta_{12} \, \dot{\theta}_1, \\
\dot{x}_2 &= v_2 \cos\theta_2, \quad \dot{y}_2 = v_2 \sin\theta_2, \\
\dot{v}_0 &= \dot{v}_0^{\rm control}, \quad \dot{\delta}_0 = \dot{\delta}_0^{\rm control}
\end{aligned}
```

where $L_i$ and $M_i$ are the vehicle and trailer geometric parameters.

### Objective

The goal is to **minimize the final time** while reducing large trailer articulation angles:

```math
J = t_f + \int_0^{t_f} (\beta_{01}^2(t) + \beta_{12}^2(t)) \, dt \to \min
```

### References

- Vanroye, L., Sathya, A., De Schutter, J., & Decré, W. (2023). FATROP: A Fast Constrained Optimal Control Problem Solver for Robot Trajectory Optimization and Control. *arXiv preprint arXiv:2303.16746*. Retrieved from https://arxiv.org/pdf/2303.16746
- Kretzschmar, H., & Burgard, W. (2019). Optimal motion planning for truck and trailer systems: A review. *IEEE Transactions on Intelligent Vehicles*, 4(3), 256–271.  
- Falcone, P., Borrelli, F., Asgari, J., Tseng, H. E., & Hrovat, D. (2007). Predictive active steering control for autonomous vehicle systems. *IEEE Transactions on Control Systems Technology*, 15(3), 566–580.  