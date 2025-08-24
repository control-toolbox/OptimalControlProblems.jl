This problem, introduced in the COPS collection (More et al., 2001), involves the **minimum-time motion of a chain-like system**. The objective is to transfer the chain from a given initial configuration to a target final configuration while minimizing the vertical displacement of one of the states.  
This classical problem (see Cesari [10, pages 126–127]) was suggested by Hans Mittelmann.

### System Dynamics

The system has three states and one control:

- $x_1$ : horizontal position  
- $x_2$ : vertical position (to be minimized at final time)  
- $x_3$ : chain length coordinate  
- $u$ : control input (horizontal velocity)  

The dynamics are expressed as:

```math
\dot{x}_1 = u
```

```math
\dot{x}_2 = x_1 \sqrt{1 + u^2}
```

```math
\dot{x}_3 = \sqrt{1 + u^2}
```

Here, $x_1$ appears in the dynamics of $x_2$, introducing a nonlinear coupling between horizontal and vertical motion. The variable $x_3$ measures the chain extension, which grows with the control magnitude.

### Boundary Conditions

- **Initial conditions**:

```math
x_1(0) = a, \quad x_2(0) = 0, \quad x_3(0) = 0
```

- **Final conditions**:

```math
x_1(T) = b, \quad x_3(T) = L
```

- No explicit constraint is placed on $x_2(T)$, but it is the target of minimization.

### Objective

The goal is to **minimize the vertical displacement** $x_2(T)$ at the final time $T$:

```math
J = x_2(T) \to \min
```

subject to the system dynamics and boundary conditions.

### References

- More, J., Garbow, B., Hillstrom, K., & Watson, L. (2001). *COPS: Constrained Optimization Problem Set* (COPS3). Mathematics and Computer Science Division, Argonne National Laboratory. Retrieved from https://www.mcs.anl.gov/~more/cops/cops3.pdf  
- Cesari, L. (1983). *Optimization – Theory and Applications. Problems with Ordinary Differential Equations*. Springer-Verlag.  
- Mittelmann, H. (suggested the hanging chain problem, as cited in COPS3).
