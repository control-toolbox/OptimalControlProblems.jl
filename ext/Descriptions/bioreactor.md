The **photobioreactor–digester system problem** is a benchmark in constrained optimal control.  
It models the coupled dynamics of a microalgae photobioreactor and an anaerobic digester for methane production.  
The system includes three state variables: the algae concentration $y(t)$, the substrate concentration $s(t)$, and the biomass concentration $b(t)$.  
The control variable $u(t)$ represents the input flow rate between the two units.  
The goal is to maximise methane production over a fixed horizon while satisfying biological and operational constraints [Bayen et al. 2014](https://doi.org/10.1002/oca.2127).

### Mathematical formulation

The problem can be stated as

```math
\begin{aligned}
\min_{y,s,b,u} \quad & J(y,s,b,u) = - \frac{1}{\beta+c} \int_0^T \mu_2(s(t))\, b(t) \,\mathrm{d}t \\[1em]
\text{s.t.} \quad &
\dot{y}(t) = \frac{\mu(t)\, y(t)}{1+y(t)} - (r+u(t))\, y(t), \\[0.5em]
& \dot{s}(t) = -\mu_2(s(t))\, b(t) + u(t)\, \beta\big(\gamma y(t) - s(t)\big), \\[0.5em]
& \dot{b}(t) = \big(\mu_2(s(t)) - u(t)\, \beta\big)\, b(t), \\[0.5em]
& 0 \le u(t) \le 1, \\[0.5em]
& y(t) \ge 0,\; s(t) \ge 0,\; b(t) \ge 10^{-3}, \\[0.5em]
& 0.05 \le y(0) \le 0.25,\;\; 0.5 \le s(0) \le 5,\;\; 0.5 \le b(0) \le 3.
\end{aligned}
```

The horizon is fixed to $T = 200$ (rescaled units), corresponding to several day–night cycles.

The model components are:  

- **Light model:** $\mu(t) = \mu_{\text{bar}} \,\max\!\big(0,\sin(\tau(t))\big)^2$, where $\tau(t)$ encodes the periodic day–night cycle,  
- **Growth function (Monod law):** $\mu_2(s) = \mu_2^m \,\dfrac{s}{K_s + s}$.

### Parameter values

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Flow coupling between reactors | $\beta$ | 1 |
| Cost scaling | $c$ | 2 |
| Substrate–algae interaction | $\gamma$ | 1 |
| Half-period of light cycle | $\text{halfperiod}$ | 5 |
| Monod half-saturation constant | $K_s$ | 0.05 |
| Maximum biomass growth rate | $\mu_2^m$ | 0.1 |
| Maximum light intensity | $\mu_{\text{bar}}$ | 1 |
| Algal decay rate | $r$ | 0.005 |
| Time horizon | $T$ | 200 |

### Qualitative behaviour

The optimal solution exploits the **periodic light–dark structure**:  
algal growth is favoured during illuminated phases, while methane production dominates during dark phases.  
The control $u(t)$ typically exhibits a **bang–bang structure**, alternating between minimum and maximum values, with possible **singular arcs** when substrate and biomass reach balanced levels.  
The positivity constraint on biomass $b(t)$ is usually active at the beginning of the process, shaping the initial control action.

### Characteristics

- Nonlinear coupled dynamics with periodic forcing,  
- Control and state constraints ensuring feasibility,  
- Bang–bang and singular arc structures in the optimal control,  
- Serves as a benchmark for optimal control methods with periodic and constrained systems.

### References

- Bayen, T., Mairet, F., Martinon, P., & Sebbah, M. (2014). *Analysis of a periodic optimal control problem connected to microalgae anaerobic digestion*. Optimal Control Applications and Methods. [https://doi.org/10.1002/oca.2127](https://doi.org/10.1002/oca.2127)  
  This paper analyzes a periodic optimal control problem modeling a coupled microalgae photobioreactor and anaerobic digester. It provides theoretical insights and numerical solutions for maximizing methane production under biological and operational constraints.

- BOCOP examples: Photobioreactor–digester system problem. [https://project.inria.fr/bocop/files/2017/05/Examples-BOCOP.pdf](https://project.inria.fr/bocop/files/2017/05/Examples-BOCOP.pdf)  
  This example demonstrates the practical implementation of the photobioreactor–digester system in BOCOP, serving as a benchmark for constrained, nonlinear, periodic optimal control problems.