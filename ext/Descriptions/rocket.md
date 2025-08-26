This problem models the **maximisation of the final altitude of a vertically launched rocket**, a classical example in optimal control theory with **singular arcs** (see Bryson [7, pp. 392–394]).  
The rocket dynamics involve thrust, drag, gravity, and fuel consumption.

### System Description

The state variables are:

```math
x = (h, v, m)
```

- $h$: altitude  
- $v$: vertical velocity  
- $m$: mass  

The control variable is:

```math
u = T
```

- $T$: rocket thrust  

### Dynamics

The equations of motion are:

```math
\dot{h} = v
```

```math
\dot{v} = \frac{T - D(h, v) - m g(h)}{m}
```

```math
\dot{m} = -\frac{T}{c}
```

where  

- **Drag**:
```math
D(h, v) = D_c \, v^2 \exp\!\left(-h_c \, \frac{h - h_0}{h_0}\right)
```

- **Gravity**:
```math
g(h) = g_0 \left(\frac{h_0}{h}\right)^2
```

- **Fuel constant**:
```math
c = \tfrac{1}{2}\sqrt{g_0 h_0}
```

### Constraints

- **Control constraints**:
```math
0 \leq T(t) \leq T_{\max}
```

with  
```math
T_{\max} = T_c \, m_0 g_0
```

- **State constraints**:
```math
h(t) \geq h_0
```

```math
v(t) \geq v_0
```

```math
m_f \leq m(t) \leq m_0
```

- **Initial conditions**:
```math
h(0) = h_0, \quad v(0) = v_0, \quad m(0) = m_0
```

- **Final condition**:
```math
m(T) = m_f = m_c m_0
```

### Objective

The objective is to **maximise the final altitude** $h(T)$:

```math
J = -h(T) \to \min
```

### Parameters

For the standard nondimensionalised version of the problem:

```math
h_0 = 1, \quad v_0 = 0, \quad m_0 = 1, \quad g_0 = 1
```

```math
T_{\max} = 3.5 g_0 m_0, \quad D_c = \tfrac{1}{2} v_c \tfrac{m_0}{g_0}, \quad c = \tfrac{1}{2}\sqrt{g_0 h_0}
```

with  

```math
h_c = 500, \quad v_c = 620, \quad m_c = 0.6
```

### References

- Bryson, A. E. (1999). *Dynamic Optimization*. Addison Wesley Longman. (pp. 392–394)  
- More, J., Garbow, B., Hillstrom, K., & Watson, L. (2001). *COPS: Constrained Optimization Problem Set* (COPS3). Argonne National Laboratory. Retrieved from https://www.mcs.anl.gov/~more/cops/cops3.pdf  
