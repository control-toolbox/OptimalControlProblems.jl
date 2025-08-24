This problem models the **optimal descent of a glider**, inspired by the COPS collection (More et al., 2001).  
The goal is to steer a glider from a given initial altitude and velocity to a target altitude while minimizing the horizontal distance traveled, taking into account aerodynamic lift and drag forces.

### System Dynamics

The system has four states and one control:

- $x$ : horizontal position  
- $y$ : vertical position (altitude)  
- $v_x$ : horizontal velocity  
- $v_y$ : vertical velocity  
- $c_L$ : lift coefficient (control)  

The dynamics are expressed as:

```math
\dot{x} = v_x
```

```math
\dot{y} = v_y
```

```math
\dot{v}_x = - \frac{L \, w + D \, v_x}{m \, v}
```

```math
\dot{v}_y = \frac{L \, v_x - D \, w}{m \, v} - g
```

where

```math
v = \sqrt{v_x^2 + w^2}, \quad w = v_y - U(r), \quad r = \left(\frac{x}{r_0} - 2.5\right)^2
```

```math
U(r) = u_c (1 - r) e^{-r}
```

```math
D = \frac{1}{2} \rho S (c_0 + c_1 c_L^2) v^2, \quad L = \frac{1}{2} \rho S c_L v^2
```

Here, $D$ and $L$ represent drag and lift, $m$ is the mass, $g$ is gravity, $S$ is wing area, $\rho$ the air density, and $c_0$, $c_1$, $u_c$, $r_0$ are aerodynamic parameters.

### Boundary Conditions

- **Initial conditions**:

```math
x(0) = x_0, \quad y(0) = y_0, \quad v_x(0) = v_{x0}, \quad v_y(0) = v_{y0}
```

- **Final conditions**:

```math
y(T) = y_f, \quad v_x(T) = v_{xf}, \quad v_y(T) = v_{yf}, \quad T \ge 0
```

- **State constraints**:

```math
x(t) \ge 0, \quad v_x(t) \ge 0
```

- **Control constraints**:

```math
c_{L,\min} \le c_L(t) \le c_{L,\max}
```

### Objective

The goal is to **minimize the horizontal displacement** $x(T)$:

```math
J = -x(T) \to \min
```

subject to the dynamics, boundary conditions, and state/control constraints.

### References

- More, J., Garbow, B., Hillstrom, K., & Watson, L. (2001). *COPS: Constrained Optimization Problem Set* (COPS3). Mathematics and Computer Science Division, Argonne National Laboratory. Retrieved from https://www.mcs.anl.gov/~more/cops/cops3.pdf
- Cesari, L. (1983). *Optimization – Theory and Applications. Problems with Ordinary Differential Equations*. Springer-Verlag.
