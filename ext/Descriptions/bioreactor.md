This problem models a coupled photobioreactor–digester system for methane production.  
The system consists of three state variables: the algae concentration $y(t)$, the substrate concentration $s(t)$, and the biomass concentration $b(t)$.  
The control variable $u(t)$ represents the input flow rate between the two units.  
The dynamics include algal growth driven by light, substrate consumption, and biomass evolution.  
The aim is to maximise methane production over a fixed time horizon under biological and operational constraints.

### Mathematical formulation

We minimise

```math
\min_{y,\,s,\,b,\,u} J(y,s,b,u) = - \frac{1}{\beta+c} \int_0^T \mu_2(s(t))\, b(t)\, dt,
```

subject to the dynamics
```math
\dot{y}(t) = \frac{\mu(t)\, y(t)}{1+y(t)} - (r+u(t))\, y(t),
```
```math
\dot{s}(t) = -\mu_2(s(t))\, b(t) + u(t)\, \beta\big(\gamma y(t) - s(t)\big),
```
```math
\dot{b}(t) = \big(\mu_2(s(t)) - u(t)\, \beta\big)\, b(t),
```

with  
- **Light model:** $\mu(t) = \mu_{\text{bar}} \,\max(0,\sin(\tau(t)))^2$, where $\tau(t)$ encodes a periodic day–night cycle,  
- **Growth function (Monod law):** $\mu_2(s) = \mu_2^m \,\dfrac{s}{K_s + s}$.

### Constraints

- Control bounds: $0 \leq u(t) \leq 1$,  
- State bounds: $y(t) \geq 0,\; s(t) \geq 0,\; b(t) \geq 10^{-3}$,  
- Initial conditions: $0.05 \leq y(0) \leq 0.25$, $0.5 \leq s(0) \leq 5$, $0.5 \leq b(0) \leq 3$.

The horizon is fixed to $T = 200$ (rescaled units), corresponding to several day–night periods.

### Qualitative behaviour

The optimal solution exploits the periodic structure of the problem:  
during illuminated phases, algal growth is favoured, while in dark phases methane production becomes predominant.  
The control exhibits a near bang–bang structure, alternating between minimal and maximal input flows, with possible singular arcs when substrate and biomass reach balanced levels.  
The constraints on positivity of the states are typically active for $b(t)$ at the beginning of the process.

### References

- Bayen, T., Mairet, F., Martinon, P., & Sebbah, M. (2014). *Analysis of a periodic optimal control problem connected to microalgae anaerobic digestion*. Optimal Control Applications and Methods. [DOI:10.1002/oca.2127](https://hal.archives-ouvertes.fr/hal-00860570)  
- Bayen, T., Mairet, F., Martinon, P., & Sebbah, M. (2013). *Optimising the anaerobic digestion of microalgae in a coupled process*. 13th European Control Conference.  
- Barbosa, M.J., & Wijffels, R.H. (2010). *An Outlook on Microalgal Biofuels*. Science, 329, 796–799.  
- Betts, J.T. (2001). *Practical methods for optimal control using nonlinear programming*. SIAM.  
- BOCOP repository: https://github.com/control-toolbox/bocop/tree/main/bocop
