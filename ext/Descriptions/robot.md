This problem models the **time-optimal motion of a robot arm** moving between two points, following the formulation of Mössner-Beigel (PhD thesis, Heidelberg University) and the implementation described by Vanderbei (2001).  
The arm is represented as a rigid bar of total length $L$, pivoting at the origin of a spherical coordinate system.

### System Description

- The arm protrudes a distance $\rho$ from the origin in one direction, and $L - \rho$ in the opposite direction.  
- The orientation of the arm is described by two angles:
  - $\theta$: horizontal angle from the plane, with bounds $|\theta| \leq \pi$  
  - $\varphi$: vertical angle, with bounds $0 \leq \varphi \leq \pi$  

The state variables are:

```math
x = (\rho, \dot{\rho}, \theta, \dot{\theta}, \varphi, \dot{\varphi})
```

The control inputs are:

```math
u = (u_{\rho}, u_{\theta}, u_{\varphi})
```

### Dynamics

The continuous-time dynamics are governed by the second-order system:

```math
L \ddot{\rho} = u_{\rho}
```

```math
I_{\theta} \ddot{\theta} = u_{\theta}
```

```math
I_{\varphi} \ddot{\varphi} = u_{\varphi}
```

where the moments of inertia are defined as:

```math
I_{\theta} = \big( (L - \rho)^3 + \rho^3 \big) \sin^2(\varphi)
```

```math
I_{\varphi} = (L - \rho)^3 + \rho^3
```

In first-order form, with states $(\rho, \dot{\rho}, \theta, \dot{\theta}, \varphi, \dot{\varphi})$, the dynamics become:

```math
\dot{\rho} = \dot{\rho}, \quad
\ddot{\rho} = \frac{u_{\rho}}{L}
```

```math
\dot{\theta} = \dot{\theta}, \quad
\ddot{\theta} = \frac{3u_{\theta}}{I_{\theta}}
```

```math
\dot{\varphi} = \dot{\varphi}, \quad
\ddot{\varphi} = \frac{3u_{\varphi}}{I_{\varphi}}
```

### Constraints

- **State constraints**:
```math
0 \leq \rho(t) \leq L
```

```math
-\pi \leq \theta(t) \leq \pi
```

```math
0 \leq \varphi(t) \leq \pi
```

- **Control constraints**:
```math
|u_{\rho}(t)| \leq 1, \quad |u_{\theta}(t)| \leq 1, \quad |u_{\varphi}(t)| \leq 1
```

- **Initial conditions**:
```math
\rho(0) = 4.5, \quad \theta(0) = 0, \quad \varphi(0) = \pi/4
```

```math
\dot{\rho}(0) = \dot{\theta}(0) = \dot{\varphi}(0) = 0
```

- **Final conditions**:
```math
\rho(T) = 4.5, \quad \theta(T) = \tfrac{2\pi}{3}, \quad \varphi(T) = \pi/4
```

```math
\dot{\rho}(T) = \dot{\theta}(T) = \dot{\varphi}(T) = 0
```

### Objective

The objective is to **minimise the transfer time** $T$:

```math
J = T \to \min
```

### Remarks

This model is simplified, as it ignores the non-inertial nature of the spherical coordinate frame (i.e., Coriolis and centrifugal forces are not included).  
Vanderbei’s implementation eliminates the controls $u$ by substitution, transforming the equations of motion into inequality constraints on accelerations.

### References

- Mössner-Beigel, M. (1989). *Thesis*, Heidelberg University.  
- Vanderbei, R. J. (2001). *Case studies in trajectory optimization: Trains, Planes, and Other Pastimes*.  
- More, J., Garbow, B., Hillstrom, K., & Watson, L. (2001). *COPS: Constrained Optimization Problem Set* (COPS3). Argonne National Laboratory. Retrieved from https://www.mcs.anl.gov/~more/cops/cops3.pdf  
