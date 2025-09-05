The **Electric Vehicle (EV) trajectory problem** is a benchmark in optimal control for energy-efficient driving.  
It was introduced by Petit & Sciarretta (2011) as a simplified longitudinal model of an electric vehicle moving along a road with varying slope.  
The state variables are the vehicle position $x(t)$ and velocity $v(t)$, and the control $u(t)$ represents the traction/braking command.  
The objective is to **minimise a combination of mechanical energy consumption and control effort**, subject to boundary conditions on the trip distance and vehicle velocity over a fixed horizon.  

Here, we denote the final time as $t_f$, with $t_f = 1$ s.

---

### Mathematical formulation

```math
\begin{aligned}
\min_{x, v, u} \quad & J(x, v, u) = \int_0^{t_f} \big( b_1 \, u(t) \, v(t) + b_2 \, u(t)^2 \big) \, \mathrm{d}t \\[0.5em]
\text{s.t.} \quad &
\dot{x}(t) = v(t), \quad
\dot{v}(t) = h_1 \, u(t) - h_2 \, v(t)^2 - h_0 - r(x(t)), \\[0.5em]
& x(0) = 0, \quad v(0) = 0, \quad x(t_f) = D, \quad v(t_f) = 0,
\end{aligned}
```

where the road slope is modelled as a cubic polynomial

```math
r(x) = \alpha_0 + \alpha_1 x + \alpha_2 x^2 + \alpha_3 x^3,
```

and the parameters $h_0$, $h_1$, $h_2$, $b_1$, $b_2$, and $\alpha_i$ define vehicle dynamics, drag, and slope effects.

---

### System parameters

| Parameter | Symbol | Value | Description |
|-----------|--------|-------|-------------|
| Trip distance | $D$ | 10 m | Total length of the trajectory |
| Friction term | $h_0$ | 0.1 | Constant resistance |
| Acceleration gain | $h_1$ | 1 | Input scaling for acceleration |
| Quadratic velocity damping | $h_2$ | 1e-3 | Air drag effect |
| Energy weight | $b_1$ | 1 | Linear contribution of input power |
| Control weight | $b_2$ | 1 | Quadratic penalty on control effort |
| Road profile | $\alpha_0,\dots,\alpha_3$ | 3, 0.4, -1, 0.1 | Cubic coefficients modelling road slope |
| Final time | $t_f$ | 1 s | Duration of the trajectory |

---

### Qualitative behaviour

- The optimal solution balances **traction and braking** to respect the velocity boundary conditions while minimising energy consumption.  
- Road slope variations $r(x)$ introduce nonlinearities, strongly influencing the structure of the optimal control.  
- For flat roads, the solution often exhibits a **bang–singular–bang profile**, while slopes lead to asymmetric trajectories.  

---

### Characteristics

- Nonlinear second-order dynamics including drag, gravity, and slope effects.  
- Mixed linear–quadratic cost reflecting energy and smoothness trade-offs.  
- Fixed-time ($t_f = 1$ s), fixed-distance trip with zero final velocity.  
- Benchmark for testing energy-optimal trajectory generation and direct transcription methods in electric vehicles.

---

### References

- **Petit, N., & Sciarretta, A. (2011).** *Optimal drive of electric vehicles using an inversion-based trajectory generation approach*. IFAC Proceedings Volumes, 44(1), 14519–14526.  
  Introduces the EV optimal control formulation, including energy-efficient trajectory generation and numerical solution methods.

- **Sciarretta, A., & Guzzella, L. (2007).** *Control of Hybrid Electric Vehicles*. IEEE Control Systems Magazine, 27(2), 60–70.  
  Provides context for energy-optimal vehicle control and modelling of longitudinal dynamics.

- **Feng, X., & Petrovic, D. (2010).** *Energy-optimal control of electric vehicles: A review of recent advances*. IEEE Transactions on Intelligent Transportation Systems, 11(3), 678–689.  
  Surveys numerical methods for EV trajectory optimisation, including handling of road slopes and energy costs.
