The **Aircraft Balanced Field Length Calculation** is a classic aeronautical engineering problem used to determine the minimum runway length required for a safe takeoff or a safe stop in the event of an engine failure.  
This implementation focuses on the **takeoff climb** phase with one engine out, starting from the critical engine failure speed $V_1$.  
The parameters are those of a mid-size jet aircraft, the nominal values of the Dymos example.

### Mathematical formulation

The problem is to minimise the final range $r(t_f)$ to reach the screen height (usually 35 ft).  
The state vector is $x(t) = (r(t), v(t), h(t), \gamma(t))$ and the control is the angle of attack $\alpha(t)$.

```math
\begin{aligned}
\min_{\alpha, t_f} \quad & r(t_f) \\
\text{s.t.} \quad & \dot{r}(t) = v(t) \cos \gamma(t), \\
& \dot{v}(t) = \frac{T \cos \alpha(t) - D}{m} - g \sin \gamma(t), \\
& \dot{h}(t) = v(t) \sin \gamma(t), \\
& \dot{\gamma}(t) = \frac{T \sin \alpha(t) + L}{m v(t)} - \frac{g \cos \gamma(t)}{v(t)}, \\
& h(t_f) = 10.668 \text{ m (35 ft)}, \\
& \gamma(t_f) = 5^\circ.
\end{aligned}
```

The aerodynamic forces $L$ and $D$ depend on the angle of attack $\alpha$, velocity $v$, and altitude $h$ (ground effect).

### Characteristics

- Nonlinear dynamics, with a ground effect in the drag model ($K$ coefficient).
- Boundary conditions representing the $V_1$ conditions and the final climb requirements.

### References

- **Dymos Documentation**. [*Aircraft Balanced Field Length Calculation*.](https://openmdao.github.io/dymos/examples/balanced_field/balanced_field.html).
  Illustrates the full multi-phase BFL calculation using Dymos and OpenMDAO.

- **Betts, J. T. (2010)**. *Practical Methods for Optimal Control and Estimation Using Nonlinear Programming*. SIAM.  
  Discusses takeoff and landing trajectory optimization in Chapter 6.
