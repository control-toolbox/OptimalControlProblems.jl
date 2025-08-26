The *double oscillator problem* is a benchmark in constrained optimal control illustrating the control of coupled mechanical systems with damping and stiffness effects. It consists of two masses connected by springs and a damper, with one mass directly influenced by an external periodic force and the other influenced indirectly through the coupling and a controlled damping term. Both the state trajectory \(x(\cdot)\) and the control \(u(\cdot)\) are decision variables. The aim is to minimise a quadratic cost that balances state deviations and control effort, subject to input constraints and the system dynamics.

The problem can be formulated as

```math
\min_{x, u} J(x,u) = 0.5 \int_0^{T} \big( x_1(t)^2 + x_2(t)^2 + u(t)^2 \big) \, dt
```

subject to the dynamics

```math
\dot{x}_1 = x_3, \qquad
\dot{x}_2 = x_4,
```

```math
\dot{x}_3 = -\frac{k_1 + k_2}{m_1} x_1 + \frac{k_2}{m_1} x_2 + \frac{1}{m_1} F(t), \qquad
\dot{x}_4 = \frac{k_2}{m_2} x_1 - \frac{k_2}{m_2} x_2 - \frac{c(1-u)}{m_2} x_4,
```

with boundary conditions

```math
x_1(0) = 0, \quad x_2(0) = 0
```

and the control constraint

```math
-1 \le u(t) \le 1,
```

where \(F(t) = \sin\left(\frac{2\pi}{T} t\right)\) is a prescribed periodic forcing term and \(T = 2\pi\).

### Qualitative behaviour

- The system exhibits coupled oscillatory dynamics due to the interaction of the two masses through the springs and damper.  
- The control input modulates the damping of the second mass, influencing the amplitude and phase of its oscillations.  
- The quadratic cost penalises both deviations of the masses and the control effort, leading to a trade-off between precise tracking of zero displacement and energy usage.

### Characteristics

- Coupled linear–nonlinear dynamics with external periodic forcing.  
- Control input enters through a damping term, introducing a nonlinearity in the system response.  
- Bounded control with symmetric limits.  
- Widely used to benchmark numerical methods for constrained optimal control in mechanical systems.

### References

- Graichen, K., & Petit, N. (2009). *Incorporating a class of constraints into the dynamics of optimal control problems.* Optim. Control Appl. Methods.  
- MathMod 2018: Examples on coupled mechanical systems and damping control. [Link to source](https://cas.minesparis.psl.eu/~petit/papers/mathmod2018/main.pdf)
