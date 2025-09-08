The **Goddard rocket problem** is a classical optimal control problem that models the ascent of a vertically launched rocket.  
The objective is to **maximise the final altitude** by optimally controlling the thrust, while accounting for atmospheric drag, gravity, and fuel consumption.  
This problem is notable for its inclusion of **singular arcs**, where the control is neither at its maximum nor minimum, complicating the solution process.  
Historically, it was first proposed by **Robert H. Goddard** in 1919 (*A Method of Reaching Extreme Altitudes*), one of the founding works in modern rocketry.

### Mathematical formulation

The problem can be stated as

```math
\begin{aligned}
\min_{h,v,m,T} \quad & J = -h(T) \\[0.5em]
\text{s.t.} \quad
\dot{h}(t) &= v(t), \\[0.25em]
\dot{v}(t) &= \frac{T(t) - D(h(t), v(t)) - m(t) g(h(t))}{m(t)}, \\[0.25em]
\dot{m}(t) &= -\frac{T(t)}{c}, \\[0.25em]
0 \le T(t) &\le T_{\max}, \\[0.25em]
h(t) &\ge h_0, \\[0.25em]
v(t) &\ge v_0, \\[0.25em]
m_f \le m(t) &\le m_0, \\[0.25em]
h(0) &= h_0, \quad v(0) = v_0, \quad m(0) = m_0, \\[0.25em]
m(T) &= m_f,
\end{aligned}
```

where  

- **Drag**:  
```math
D(h,v) = D_c \, v^2 \exp\!\left(-h_c \frac{h - h_0}{h_0}\right)
```

- **Gravity**:  
```math
g(h) = g_0 \left(\frac{h_0}{h}\right)^2
```

- **Fuel constant**:  
```math
c = \frac{1}{2} \sqrt{g_0 h_0}
```

- **Maximum thrust**:  
```math
T_{\max} = T_c m_0 g_0
```

### Parameters

| Parameter                  | Symbol | Value                  |
|----------------------------|--------|------------------------|
| Initial altitude           | $h_0$  | 1                      |
| Initial velocity           | $v_0$  | 0                      |
| Initial mass               | $m_0$  | 1                      |
| Gravitational constant     | $g_0$  | 1                      |
| Thrust coefficient         | $T_c$  | 3.5                    |
| Drag coefficient           | $D_c$  | $\frac{1}{2} v_c \frac{m_0}{g_0}$ |
| Characteristic altitude    | $h_c$  | 500                    |
| Characteristic velocity    | $v_c$  | 620                    |
| Characteristic mass ratio  | $m_c$  | 0.6                    |
| Final mass                 | $m_f$  | $m_c m_0$              |

### Qualitative behaviour

The optimal trajectory often exhibits a **bang–singular–bang structure**, with thrust at maximum, then along a singular arc, and finally at minimum.  
The singular arc arises from the interplay of drag and gravity, producing a non-trivial optimal control law.  
This behaviour illustrates the complexity of the problem and the challenges in numerically resolving singular arcs.

### Characteristics

- Nonlinear coupled dynamics with drag and gravity,  
- Singular arcs in the optimal control,  
- State and control constraints,  
- Classical benchmark for testing optimal control algorithms, especially in aerospace applications.

### References

- Goddard, R. H. (1919). *A Method of Reaching Extreme Altitudes*. Smithsonian Institution, Washington D.C.  
  The original formulation of the vertical rocket ascent problem, introducing fuel consumption, gravity, and drag effects.  

- Bryson, A. E. (1999). *Dynamic Optimization*. Addison Wesley Longman. pp. 392–394.  
  Provides an introduction to singular arcs in optimal control with the Goddard rocket as a primary example.  

- Garfinkel, B. (1963). A Solution of the Goddard Problem. *SIAM Journal on Control*, 1(1), 1–20. [doi:10.1137/0301020](https://epubs.siam.org/doi/10.1137/0301020)  
  Analytical and numerical solutions of the Goddard problem highlighting singular arc structures.

- Tsiotras, P., & Kelley, H. J. (1992). Goddard Problem with Constrained Time of Flight. *Journal of Guidance, Control, and Dynamics*, 15(2), 394–399. [doi:10.2514/3.20836](https://doi.org/10.2514/3.20836)  
  Study of time-constrained versions of the Goddard problem and associated optimal control strategies.

- Bonnans, F., Martinon, P., & Trélat, E. (2007). Singular Arcs in the Generalized Goddard's Problem. [arXiv:math/0703911](https://arxiv.org/pdf/math/0703911)  
  Analysis of singular arcs and numerical methods for the Goddard problem.  

- More, J., Garbow, B., Hillstrom, K., & Watson, L. (2001). *COPS: Constrained Optimization Problem Set* (COPS3). Argonne National Laboratory. [Link](https://www.mcs.anl.gov/~more/cops/cops3.pdf)  
  Provides a benchmark implementation of the Goddard rocket problem for testing numerical optimization methods.
