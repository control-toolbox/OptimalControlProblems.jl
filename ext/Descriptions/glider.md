The **hang glider problem** is a classical benchmark in optimal control.  
It consists of steering a hang glider from an initial horizontal position and altitude to a target altitude while maximising the **horizontal distance travelled**.  
The glider dynamics incorporate lift, drag, gravity, and the effect of a thermal updraft.  
The control variable is the lift coefficient $c_L$, which modulates the aerodynamic lift and influences the trajectory through the thermal region.

### Mathematical formulation

```math
\begin{aligned}
\min_{x, y, v_x, v_y, c_L, t_f} \quad & -x(t_f) \\[1em]
\text{s.t.} \quad &
\dot{x} = v_x, \quad
\dot{y} = v_y, \\[0.25em]
& \dot{v}_x = -\frac{L \, w + D \, v_x}{m \, v}, \quad
\dot{v}_y = \frac{L \, v_x - D \, w}{m \, v} - g, \\[1em]
& c_L^{\min} \le c_L(t) \le c_L^{\max}, \quad x(t) \ge 0, \quad v_x(t) \ge 0, \\[1em]
& (x, y, v_x, v_y)|_{t=0} = (x_0, y_0, v_{x0}, v_{y0}), \quad
(y, v_x, v_y)|_{t=t_f} = (y_f, v_{xf}, v_{yf}),
\end{aligned}
```

with  

```math
v = \sqrt{v_x^2 + w^2}, \quad
w = v_y - U_\text{updraft}(x), \quad
L = \frac{1}{2} \rho S c_L v^2, \quad
D = \frac{1}{2} \rho S (c_0 + c_1 c_L^2) v^2,
```

and  

```math
U_\text{updraft}(x) = u_c\, (1 - r) e^{-r}, \quad r = \left( \frac{x}{r_0} - 2.5 \right)^2,
```

where $m, g, S, \rho, c_0, c_1, u_c, r_0$ are constants describing the glider and the thermal properties.

---

### System parameters

| Parameter | Symbol | Value | Description |
|-----------|--------|-------|-------------|
| Initial horizontal position | $x_0$ | 0 | m |
| Initial altitude | $y_0$ | 1000 | m |
| Final altitude | $y_f$ | 900 | m |
| Initial horizontal velocity | $v_{x0}$ | 13.23 | m/s |
| Final horizontal velocity | $v_{xf}$ | 13.23 | m/s |
| Initial vertical velocity | $v_{y0}$ | -1.288 | m/s |
| Final vertical velocity | $v_{yf}$ | -1.288 | m/s |
| Lift coefficient bounds | $c_L$ | [0, 1.4] | Control input |
| Final time | $t_f$ | free | s |

---

### Qualitative behaviour

- The optimal trajectory exploits the thermal updraft to maximise horizontal distance.  
- The lift coefficient $c_L$ balances altitude loss and horizontal progression.  
- The dynamics are nonlinear due to the coupling of lift, drag, and relative velocity in the thermal.  
- Horizontal velocity is maintained positive, and the trajectory respects the control and state constraints.

---

### Characteristics

- Nonlinear four-dimensional dynamics with one control input.  
- Free final time.  
- Mixed state and control constraints.  
- Widely used as a benchmark for trajectory optimisation and direct transcription methods in nonlinear optimal control.

---

### References

- **Dolan, E. D., & More, J. J. (2004).** *Benchmarking Optimization Software with COPS 3.0*. Argonne National Laboratory.  
  [PDF](https://www.mcs.anl.gov/~more/cops/cops3.pdf)  
  Includes the hang glider problem in the COPS 3.0 benchmark collection with problem formulation and solver comparisons.

- **PSOPT Example: Hang Glider Problem.** [psopt.net/list-of-examples](https://www.psopt.net/list-of-examples)  
  Demonstrates a practical implementation of the hang glider optimal control problem in PSOPT.

- **PSOPT GitHub Repository: Hang Glider Example.** [github.com/PSOPT/psopt/blob/master/examples/glider/glider.cxx](https://github.com/PSOPT/psopt/blob/master/examples/glider/glider.cxx)  
  Source code example for testing direct transcription and NLP solvers with the hang glider problem.
