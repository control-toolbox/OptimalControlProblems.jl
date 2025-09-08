The **insurance problem** is a benchmark in optimal control inspired by health and financial economics.  
It consists of controlling insurance coverage, expenses, revenue, health, and utility over a fixed horizon to **maximise expected utility**.  
The system dynamics model the evolution of insurance participation, expenses, and auxiliary variables, while respecting bounds on all state and control variables.  
The controls represent health investments, insurance revenue, health outcomes, and utility contributions.

---

### Mathematical formulation

The **objective** is to maximise expected utility:

```math
\max_{I, m, x_3, h, R, H, U, dU/dR, P} \quad 
\int_0^{t_f} U(t) \, f_x(t) \, dt
```

where $f_x(t)$ is the probability density of illness events.

---

#### State, control, and auxiliary bounds

The variables are subject to:

```math
\begin{aligned}
\text{State bounds:} & \quad 0 \le I(t) \le 1.5, \quad 0 \le m(t) \le 1.5, \quad 0 \le x_3(t), \\
\text{Control bounds:} & \quad 0 \le h(t) \le 25, \\
\text{Auxiliary bounds:} & \quad 0 \le R(t), \quad 0 \le H(t), \quad 0.001 \le \frac{dU}{dR}(t), \quad 0 \le P.
\end{aligned}
```

---

#### Initial and terminal conditions

```math
(I, m, x_3)|_{t=0} = (0, 0.001, 0), \quad 
P - x_3(t_f) = 0
```

---

#### Dynamics

```math
\dot{x}(t) =
\begin{bmatrix}
\dot{I}(t) \\[1mm]
\dot{m}(t) \\[1mm]
\dot{x}_3(t)
\end{bmatrix} =
\begin{bmatrix}
(1 - \gamma t \, v'/dU/dR(t)) \, h(t) \\[1mm]
h(t) \\[1mm]
(1 + \sigma) I(t) f_x(t)
\end{bmatrix}
```

with the **expense transformation**:

```math
v = \frac{m(t)^{\alpha/2}}{1 + m(t)^{\alpha/2}}, \quad
v' = \frac{\alpha}{2} \frac{m(t)^{\alpha/2 - 1}}{(1 + m(t)^{\alpha/2})^2}.
```

---

#### Algebraic / auxiliary constraints

```math
\begin{aligned}
R(t) &= w - P + I(t) - m(t) - \varepsilon(t), \\
H(t) &= h_0 - \gamma t (1 - v), \\
U(t) &= 1 - e^{-s R(t)} + H(t), \\
\frac{dU}{dR}(t) &= s \, e^{-s R(t)}, \\
f_x(t) &= \lambda e^{-\lambda t} + \frac{e^{-\lambda t_f}}{t_f}, \\
\varepsilon(t) &= k \frac{t}{t_f - t + 1}.
\end{aligned}
```

These constraints encode **revenue balance**, **health evolution**, **utility computation**, and **auxiliary relationships**.

---

### System parameters

| Parameter | Symbol | Value | Description |
|-----------|--------|-------|-------------|
| Risk aversion | $\gamma$ | 0.2 | Health cost scaling |
| Illness rate | $\lambda$ | 0.25 | Hazard parameter |
| Baseline health | $h_0$ | 1.5 | Initial health level |
| Weight factor | $w$ | 1 | Revenue baseline |
| Sensitivity | $s$ | 10 | Utility sensitivity to revenue |
| Exponent | $\alpha$ | 4 | Expense nonlinearity |
| Expense rate | $k$ | 0 | Time-varying adjustment |
| Shock | $\sigma$ | 0 | Auxiliary multiplier |
| Final time | $t_f$ | 10 | s |

---

### Qualitative behaviour

- The optimal strategy balances **insurance coverage**, **health investment**, and **expenses** to maximise utility under constraints.  
- Revenue and health constraints link controls and states nonlinearly.  
- The dynamics incorporate moral hazard effects and diminishing returns on expenses.  
- Auxiliary variables track utility and revenue derivatives, ensuring consistent constraints.

---

### Characteristics

- Nonlinear three-dimensional state dynamics with five control variables and one auxiliary.  
- Fixed final time with state and control bounds.  
- Models health insurance optimisation under ex post moral hazard and risk-sensitive utility.  
- Serves as a benchmark for testing constrained nonlinear OCP solvers.

---

### References

- **Martinon, P., Picard, P., & Raj, A. (2018).** *On the design of optimal health insurance contracts under ex post moral hazard*.  
  The Geneva Risk and Insurance Review, 43, 127–155. [doi:10.1057/s10713-018-0034-y](https://link.springer.com/article/10.1057/s10713-018-0034-y)  
  Provides a theoretical framework for optimal health insurance design under moral hazard, which inspired the benchmark problem formulation.

- **Bocop Repository: Insurance Example.** [github.com/control-toolbox/bocop/tree/main/bocop](https://github.com/control-toolbox/bocop/tree/main/bocop)  
  Contains the implementation of the insurance optimal control problem used for testing direct transcription methods and NLP solvers.
