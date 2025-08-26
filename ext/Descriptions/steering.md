This problem models the **time-optimal steering of a vehicle** with bounded control inputs, inspired by the COPS collection (More et al., 2001).  
The goal is to move the system from an initial state to a specified final state in minimum time while respecting constraints on the steering input.

### System Dynamics

The system has four states and one control:

- $x_1$ : horizontal position  
- $x_2$ : vertical position  
- $x_3$ : velocity in $x_1$ direction  
- $x_4$ : velocity in $x_2$ direction  
- $u$ : steering angle (control)

The dynamics are expressed as:

```math
\dot{x}_1 = x_3
```

```math
\dot{x}_2 = x_4
```

```math
\dot{x}_3 = a \cos(u)
```

```math
\dot{x}_4 = a \sin(u)
```

where $a$ is a constant acceleration parameter, and $u$ is constrained by:

```math
u_{\min} \le u(t) \le u_{\max}
```

### Boundary Conditions

- **Initial conditions**:

```math
x(0) = x_s
```

- **Final conditions**:

```math
x(T) = x_f, \quad T \ge 0
```

- **Control constraints**:

```math
u_{\min} \le u(t) \le u_{\max}
```

### Objective

The goal is to **minimize the final time** $T$:

```math
J = T \to \min
```

subject to the dynamics, boundary conditions, and control constraints.

### References

- More, J., Garbow, B., Hillstrom, K., & Watson, L. (2001). *COPS: Constrained Optimization Problem Set* (COPS3). Mathematics and Computer Science Division, Argonne National Laboratory. Retrieved from https://www.mcs.anl.gov/~more/cops/cops3.pdf  
- Cesari, L. (1983). *Optimization – Theory and Applications. Problems with Ordinary Differential Equations*. Springer-Verlag.
