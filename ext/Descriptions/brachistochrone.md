The **Brachistochrone problem** is a classical benchmark in the history of calculus of variations and optimal control.  
It seeks the shape of a curve (or wire) connecting two points $A$ and $B$ within a vertical plane, such that a bead sliding frictionlessly under the influence of uniform gravity travels from $A$ to $B$ in the shortest possible time.  
Originating from the challenge posed by Johann Bernoulli in 1696 [Bernoulli 1696](https://doi.org/10.1002/andp.19163551307), it marks the birth of modern optimal control theory.  
The problem involves nonlinear dynamics where the state includes position and velocity, and the control is the path's angle, with the final time $t_f$ being a decision variable to be minimised.

### Mathematical formulation

The problem can be stated as a free final time optimal control problem:

```math
\begin{aligned}
\min_{u(\cdot), t_f} \quad & J = t_f = \int_0^{t_f} 1 \,\mathrm{d}t \\[1em]
\text{s.t.} \quad & \dot{x}(t) = v(t) \sin u(t), \\[0.5em]
& \dot{y}(t) = v(t) \cos u(t), \\[0.5em]
& \dot{v}(t) = g \cos u(t), \\[0.5em]
& x(0) = x_0, \; y(0) = y_0, \; v(0) = v_0, \\[0.5em]
& x(t_f) = x_f, \; y(t_f) = y_f,
\end{aligned}
```

where $u(t)$ represents the angle of the tangent to the curve with respect to the vertical axis, and $g$ is the gravitational acceleration.

### Qualitative behaviour

This problem is a classic example of **minimum time control** with nonlinear dynamics.  
Unlike the shortest path problem (a straight line), the optimal trajectory balances the need to minimize path length with the need to maximize velocity.

The analytical solution to this problem is a **cycloid**—the curve traced by a point on the rim of a circular wheel as the wheel rolls along a straight line without slipping.  
Specifically:

- **Energy Conservation**: Since the system is conservative and frictionless, the speed at any height $h$ is given by $v = \sqrt{2gh}$ (assuming start from rest). This implies that maximizing vertical drop early in the trajectory increases velocity for the remainder of the path.
- **Concavity**: The optimal curve is concave up. It starts steeply (vertical tangent if $v_0=0$) to gain speed quickly, then flattens out near the bottom before potentially rising again to reach the target.
- **Singularity**: At the initial point, if $v_0=0$, the control is theoretically singular as the bead has no velocity to direct. Numerical solvers often require a small non-zero initial velocity or a robust initial guess to handle this start.

The control $u(t)$ is continuous and varies smoothly to trace the cycloidal arc.

### Characteristics

- **Nonlinear dynamics** involving trigonometric functions of the control.
- **Free final time** problem (time-optimal).
- Analytical solution is a **Cycloid**.
- Historically significant as the first problem solved using techniques that evolved into the Calculus of Variations.

### References

- Bernoulli, J. (1696). *Problema novum ad cujus solutionem Mathematici invitantur*.  
  Acta Eruditorum.  
  The original publication posing the challenge to the mathematicians of Europe, solved by Newton, Leibniz, L'Hôpital, and the Bernoulli brothers.

- Sussmann, H. J., & Willems, J. C. (1997). *300 years of optimal control: from the brachystochrone to the maximum principle*.  
  IEEE Control Systems Magazine. [doi.org/10.1109/37.588108](https://doi.org/10.1109/37.588108)  
  A comprehensive historical review linking the classical Brachistochrone problem to modern optimal control theory and Pontryagin's Maximum Principle.

- Dymos Examples: Brachistochrone. [OpenMDAO/Dymos](https://openmdao.github.io/dymos/examples/brachistochrone/brachistochrone.html)  
  The numerical implementation serving as the basis for the current benchmark formulation.