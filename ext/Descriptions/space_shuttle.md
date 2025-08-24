The *space shuttle reentry problem* is a classical benchmark in aerospace optimal control, originating from reentry trajectory studies (see Betts 2010, Bulirsch 1971, Dickmanns 1972). It describes the atmospheric descent of the space shuttle from high altitude to the Terminal Area Energy Management (TAEM) interface. The aim is to maximise the crossrange, i.e. the final latitude at TAEM, subject to nonlinear dynamics, control bounds, and path constraints.

The problem can be written as

```math
\max_{x,\,u} J(x,u) = \theta(t_f),
```

or equivalently

```math
\min_{x,\,u} J(x,u) = -\theta(t_f),
```

subject to the dynamics
```math
\begin{aligned}
\dot{h}(t) &= v(t)\,\sin \gamma(t), \\
\dot{\phi}(t) &= \tfrac{v(t)}{r(t)} \cos \gamma(t) \sin \psi(t) / \cos \theta(t), \\
\dot{\theta}(t) &= \tfrac{v(t)}{r(t)} \cos \gamma(t) \cos \psi(t), \\
\dot{v}(t) &= -\tfrac{D}{m} - g(t)\,\sin \gamma(t), \\
\dot{\gamma}(t) &= \tfrac{L}{m v(t)} \cos \beta(t) 
                + \cos \gamma(t)\Big(\tfrac{v(t)}{r(t)} - \tfrac{g(t)}{v(t)}\Big), \\
\dot{\psi}(t) &= \tfrac{L}{m v(t)\cos \gamma(t)} \sin \beta(t) 
                + \tfrac{v(t)}{r(t)\cos \theta(t)} \cos \gamma(t)\sin \psi(t)\sin \theta(t),
\end{aligned}
```

with aerodynamic lift and drag
```math
D = \tfrac{1}{2} c_D S \rho v^2, \qquad 
L = \tfrac{1}{2} c_L S \rho v^2,
```

where the coefficients are given by
```math
c_D = b_0 + b_1 \alpha^\circ + b_2 (\alpha^\circ)^2, \qquad
c_L = a_0 + a_1 \alpha^\circ, \qquad
\alpha^\circ = \tfrac{180}{\pi} \alpha,
```

and
```math
\rho = \rho_0 e^{-h/h_r}, \qquad
r = R_e + h, \qquad
g = \tfrac{\mu}{r^2}.
```

### Boundary conditions

At reentry interface ($t=0$):
```math
h(0) = 260{,}000 \ \text{ft}, \quad 
v(0) = 25{,}600 \ \text{ft/s}, \quad
\phi(0)=0, \quad \theta(0)=0, \quad
\gamma(0)=-1^\circ, \quad \psi(0)=90^\circ.
```

At TAEM interface ($t=t_f$):
```math
h(t_f) = 80{,}000 \ \text{ft}, \quad 
v(t_f) = 2{,}500 \ \text{ft/s}, \quad 
\gamma(t_f) = -5^\circ.
```

Final time is free within
```math
500\Delta t_{\min} \le t_f \le 500\Delta t_{\max}, \qquad 
\Delta t_{\min}=3.5, \quad \Delta t_{\max}=4.5.
```

### Constraints

- State bounds:
```math
h(t) \ge 0, \qquad 
-89^\circ \le \theta(t) \le 89^\circ, \qquad
v(t) \ge 0, \qquad 
-89^\circ \le \gamma(t) \le 89^\circ.
```

- Control bounds:
```math
-90^\circ \le \alpha(t) \le 90^\circ, \qquad 
-89^\circ \le \beta(t) \le 1^\circ.
```

### Qualitative behaviour

- The optimal trajectory balances **lift** and **drag** to control heating and deceleration while extending crossrange.  
- The **bank angle** $\beta$ determines heading changes and crossrange capability.  
- The solution typically includes a combination of steep reentry to dissipate energy and crossrange manoeuvres to reach the target latitude.  

### Characteristics

- Nonlinear, six–state dynamics with two controls.  
- Strongly nonlinear aerodynamic coefficients.  
- Free final time with bounded range.  
- Path constraints on both states and controls.  
- Widely used as a benchmark in optimal control and trajectory optimisation.  

### References

- Betts, J.T. (2010). *Practical Methods for Optimal Control and Estimation Using Nonlinear Programming*. SIAM.  
- Bulirsch, R. (1971). Numerical solution of optimal control problems with state constraints by direct methods. *Numerische Mathematik*.  
- Dickmanns, E.D. (1972). Numerical solution methods for nonlinear optimal control problems with state constraints. *Automatica*.  
- Ascher, U.M., Mattheij, R.M.M., & Russell, R.D. (1988). *Numerical Solution of Boundary Value Problems for Ordinary Differential Equations*.  
