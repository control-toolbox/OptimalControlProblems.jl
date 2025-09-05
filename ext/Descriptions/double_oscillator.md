The **Double Oscillator problem** is a benchmark in constrained optimal control illustrating the control of coupled mechanical systems with damping and stiffness effects.  
It consists of two masses connected by springs and a damper, with one mass directly influenced by an external periodic force and the other influenced indirectly through the coupling and a controlled damping term.  
Both the state trajectory $x(\cdot)$ and the control $u(\cdot)$ are decision variables.  
The aim is to minimise a quadratic cost that balances state deviations and control effort, subject to input constraints and the system dynamics.

### Mathematical formulation

The problem can be stated as

```math
\begin{aligned}
\min_{x_1, x_2, x_3, x_4, u} \quad & J(x_1, x_2, x_3, x_4, u) = 0.5 \int_0^{T} \big( x_1^2(t) + x_2^2(t) + u^2(t) \big) \, \mathrm{d}t \\[1.5em]
\text{s.t.} \quad &
\dot{x}_1 = x_3, \quad
\dot{x}_2 = x_4, \\[0.5em]
& \dot{x}_3 = -\frac{k_1 + k_2}{m_1} x_1 + \frac{k_2}{m_1} x_2 + \frac{1}{m_1} F(t), \\[0.5em]
& \dot{x}_4 = \frac{k_2}{m_2} x_1 - \frac{k_2}{m_2} x_2 - \frac{c(1-u)}{m_2} x_4, \\[1.5em]
& x_1(0) = 0, \quad x_2(0) = 0, \quad -1 \le u(t) \le 1,
\end{aligned}
```

where $F(t) = \sin\left(\frac{2 \pi}{T} t\right)$ is a prescribed periodic forcing term.

### System parameters

| Parameter | Symbol | Value | Description |
|-----------|--------|-------|-------------|
| Mass 1 | $m_1$ | 100 kg | First mass directly affected by $F(t)$ |
| Mass 2 | $m_2$ | 2 kg | Second mass influenced by damping control |
| Spring stiffness 1 | $k_1$ | 100 N/m | Spring connecting first mass to reference |
| Spring stiffness 2 | $k_2$ | 3 N/m | Coupling spring between the two masses |
| Damping coefficient | $c$ | 0.5 Ns/m | Damping affecting second mass |
| Final time | $T$ | $2\pi$ | Duration of the motion |
| Control input | $u$ | — | Modulates the damping of the second mass |

### Qualitative behaviour

- The system exhibits coupled oscillatory dynamics due to interaction between the two masses through springs and damper.  
- The control input modulates the damping of the second mass, affecting the amplitude and phase of oscillations.  
- The quadratic cost penalises both deviations of the masses and the control effort, balancing tracking and energy usage.  
- Optimal trajectories often exhibit bang–bang behaviour on $u$, consistent with theoretical results for mechanical oscillators under bounded control.

### Characteristics

- Coupled linear–nonlinear dynamics with external periodic forcing.  
- Control input enters through a damping term, introducing nonlinearity in the system response.  
- Bounded control with symmetric limits.  
- Widely used to benchmark numerical methods for constrained optimal control in mechanical systems.  
- Serves as a testbed for averaging methods and bang-bang control design.

### References

- **Coudurier, C., Lepreux, O., & Petit, N. (2018).** *Optimal bang-bang control of a mechanical double oscillator using averaging methods.* IFAC-PapersOnLine, 51(2), 49–54.  
  Investigates bang-bang control strategies for the double oscillator system using averaging methods to analyze optimal trajectories.

- **Graichen, K., & Petit, N. (2009).** *Incorporating a class of constraints into the dynamics of optimal control problems.* Optim. Control Appl. Methods, 30(5), 397–415.  
  Explores methods for integrating constraints directly into the dynamics of optimal control problems, relevant to double oscillator systems with constrained damping.

- **Petit, N. (2018).** *MathMod 2018: Examples on coupled mechanical systems and damping control.* [PDF](https://cas.minesparis.psl.eu/~petit/papers/mathmod2018/main.pdf)  
  Provides practical examples on coupled mechanical systems, including double oscillators, focusing on damping control and optimal control formulation.
