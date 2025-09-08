The **Space Shuttle reentry problem** is a classical benchmark in aerospace optimal control.  
It models the atmospheric descent of the space shuttle from high altitude to the Terminal Area Energy Management (TAEM) interface.  
The system includes six state variables: altitude $h(t)$, longitude $\phi(t)$, latitude $\theta(t)$, velocity $v(t)$, flight path angle $\gamma(t)$, and azimuth $\psi(t)$.  
The control variables are the angle of attack $\alpha(t)$ and bank angle $\beta(t)$.  
The goal is to **maximise the final latitude (crossrange) at TAEM** while satisfying aerodynamic, gravitational, and operational constraints [Betts 2010; Bulirsch 1971; Dickmanns 1972].

### Mathematical formulation

The problem can be stated as

```math
\begin{aligned}
\min_{h, \phi, \theta, v, \gamma, \psi, \alpha, \beta} \quad & J(h, \theta) = - \theta(T) \\[1em]
\text{s.t.} \quad &
\dot{h}(t) = v \sin(\gamma), \\[0.5em]
& \dot{\phi}(t) = \frac{v}{r} \cos(\gamma) \frac{\sin(\psi)}{\cos(\theta)}, \\[0.5em]
& \dot{\theta}(t) = \frac{v}{r} \cos(\gamma) \cos(\psi), \\[0.5em]
& \dot{v}(t) = -\frac{D(h,v,\alpha)}{m} - g(h) \sin(\gamma), \\[0.5em]
& \dot{\gamma}(t) = \frac{L(h,v,\alpha)}{m v} \cos(\beta) + \cos(\gamma) \left( \frac{v}{r} - \frac{g(h)}{v} \right), \\[0.5em]
& \dot{\psi}(t) = \frac{L(h,v,\alpha)}{m v \cos(\gamma)} \sin(\beta) + \frac{v}{r \cos(\theta)} \cos(\gamma) \sin(\psi) \sin(\theta), \\[0.5em]
& \alpha_{\min} \le \alpha(t) \le \alpha_{\max}, \\[0.5em]
& \beta_{\min} \le \beta(t) \le \beta_{\max}, \\[0.5em]
& h_{\min} \le h(t) \le h_{\max}, \quad
v_{\min} \le v(t) \le v_{\max}, \\[0.5em]
& \gamma_{\min} \le \gamma(t) \le \gamma_{\max}, \quad
\theta_{\min} \le \theta(t) \le \theta_{\max}, \\[0.5em]
& h(0) = h_s, \; \phi(0) = \phi_s, \; \theta(0) = \theta_s, \\[0.5em]
& v(0) = v_s, \; \gamma(0) = \gamma_s, \; \psi(0) = \psi_s, \\[0.5em]
& h(T) = h_t, \; v(T) = v_t, \; \gamma(T) = \gamma_t.
\end{aligned}
```

The final time $T$ is free but bounded.  

### Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Initial altitude | $h_s$ | $2.6 \times 10^5$ ft |
| Initial velocity | $v_s$ | $2.56 \times 10^4$ ft/s |
| Initial flight path angle | $\gamma_s$ | $-1^\circ$ |
| Initial azimuth | $\psi_s$ | $90^\circ$ |
| Final altitude | $h_t$ | $0.8 \times 10^5$ ft |
| Final velocity | $v_t$ | $0.25 \times 10^4$ ft/s |
| Final flight path angle | $\gamma_t$ | $-5^\circ$ |
| Mass | $m$ | $w/g_0$ |
| Reference area | $S$ | 2690 |
| Earth's radius | $R_e$ | 20902900 ft |
| Gravitational parameter | $\mu$ | $0.14076539 \cdot 10^{17}$ |

### Qualitative behaviour

- The optimal trajectory balances **lift** and **drag** to control heating and deceleration while extending crossrange.  
- The **bank angle** $\beta$ determines heading changes and crossrange capability.  
- The solution typically combines **steep reentry** to dissipate energy and **crossrange manoeuvres** to reach the target latitude.  

### Characteristics

- Nonlinear, six–state dynamics with two controls.  
- Strongly nonlinear aerodynamic coefficients.  
- Free final time with bounded range.  
- Path constraints on states and controls.  
- Widely used as a benchmark in optimal control and trajectory optimisation.

### References and relevance

- **Betts, J.T. (2010)**. *Practical Methods for Optimal Control and Estimation Using Nonlinear Programming*. SIAM.  
  Provides a comprehensive discussion of trajectory optimisation for aerospace vehicles, including space shuttle reentry problems. It details numerical methods applicable to nonlinear, constrained, free-final-time problems.  

- **Bulirsch, R. (1971)**. Numerical solution of optimal control problems with state constraints by direct methods. *Numerische Mathematik*.  
  Introduces early direct methods for optimal control problems with constraints, which are foundational for solving shuttle reentry trajectory problems with bounded states and controls.  

- **Dickmanns, E.D. (1972)**. Numerical solution methods for nonlinear optimal control problems with state constraints. *Automatica*.  
  Focuses on numerical techniques for nonlinear optimal control with state constraints, directly relevant to handling the shuttle’s aerodynamic and flight path restrictions.  

- **Ascher, U.M., Mattheij, R.M.M., & Russell, R.D. (1988)**. *Numerical Solution of Boundary Value Problems for Ordinary Differential Equations*.  
  Provides practical algorithms for solving boundary value problems, which underpin the solution of two-point boundary value problems like the shuttle reentry trajectory with fixed initial and terminal states.  

- **Betts, J.T. (2001)**. Survey of numerical methods for trajectory optimization. *Journal of Guidance, Control, and Dynamics*, 24(4), 643–653.  
  Reviews trajectory optimisation methods including direct transcription and collocation, which are commonly applied to the space shuttle reentry benchmark.
