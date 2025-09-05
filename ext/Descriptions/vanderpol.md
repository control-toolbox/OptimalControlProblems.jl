This problem considers the **controlled Van der Pol oscillator**, a classical nonlinear system with non-linear damping:

```math
\ddot{x}(t) - \varepsilon \, \omega \, (1 - x(t)^2) \dot{x}(t) + \omega^2 x(t) = u(t),
```

with control $u(t)$ and cost functional

```math
J(x,u) = \frac{1}{2} \int_0^{t_f} \big( x_1(t)^2 + x_2(t)^2 + u(t)^2 \big) \, dt.
```

The system is represented in first-order form as

```math
\begin{aligned}
\dot x_1(t) &= x_2(t), \\
\dot x_2(t) &= \varepsilon \, \omega \, (1 - x_1(t)^2) x_2(t) - \omega^2 x_1(t) + u(t),
\end{aligned}
```

with boundary condition

```math
x(0) = (1,0),
```

and horizon $t_f = 2$. In our simulations we use parameters $\varepsilon=1$, $\omega=1$.

### Qualitative behaviour

The Van der Pol oscillator exhibits **limit cycle behaviour** for $\varepsilon>0$, where trajectories converge to a stable periodic orbit.  
The non-linear damping leads to relaxation oscillations: slow accumulation along one branch followed by a rapid transition.  
Control input $u(t)$ allows influencing the amplitude and phase of the oscillations, and the quadratic cost penalises deviations from the origin as well as control effort.

### References

- BOCOP repository: [https://github.com/control-toolbox/bocop](https://github.com/control-toolbox/bocop)  
- Wikipedia: [Van der Pol oscillator](https://en.wikipedia.org/wiki/Van_der_Pol_oscillator)  
- Heitmann, S., Breakspear, M., *BOCOP: User Guide*, 2017–2022.  
