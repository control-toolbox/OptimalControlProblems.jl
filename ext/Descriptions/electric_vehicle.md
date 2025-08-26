The *electric vehicle problem* is a benchmark in optimal control motivated by energy-efficient driving strategies.  
It was introduced by Petit & Sciarretta (2011) as a simplified longitudinal model of an electric vehicle driving along a road with varying slope. The state is the position $x(\cdot)$ and velocity $v(\cdot)$, while the control $u(\cdot)$ represents the traction/braking command.  

The aim is to minimise a cost functional balancing *mechanical energy consumption* and *control effort*, subject to boundary conditions on the trip distance and vehicle velocity.

The problem can be written as

```math
\min_{x,\,v,\,u} J(x,v,u) = \int_0^{t_f} \big( b_1\, u(t) v(t) + b_2\, u(t)^2 \big) \, dt
```

subject to the dynamics
```math
\dot{x}(t) = v(t), \qquad
\dot{v}(t) = h_1 u(t) - h_2 v(t)^2 - h_0 - r(x(t)),
```

where $r(x)$ models the road slope as a cubic polynomial
```math
r(x) = \alpha_0 + \alpha_1 x + \alpha_2 x^2 + \alpha_3 x^3.
```

with boundary conditions
```math
x(0) = 0, \quad v(0) = 0, \qquad 
x(t_f) = D, \quad v(t_f) = 0,
```

and a fixed horizon $t_f$.

### Qualitative behaviour

- The optimal solution balances between using traction and braking to respect the velocity boundary conditions while minimising the integral cost.  
- Road slope variations $r(x)$ strongly influence the structure of the optimal control.  
- For flat roads, the solution tends to a bang–singular–bang profile, while slopes induce asymmetric profiles.  

### Characteristics

- Nonlinear second-order dynamics with drag, gravity and slope effects.  
- Mixed quadratic cost reflecting energy and smoothness trade-off.  
- Fixed-time, fixed-distance trip with zero final velocity.  
- Relevant to eco-driving and energy management in electric vehicles.  

### References

- Petit, N., & Sciarretta, A. (2011). *Optimal drive of electric vehicles using an inversion-based trajectory generation approach*. IFAC Proceedings Volumes, 44(1), 14519–14526.  
