The **Bryson–Denham problem** is a classic optimal control benchmark involving a state inequality constraint.  
It models the movement of a unit mass on a frictionless plane, where the goal is to relocate the mass over a fixed time interval with minimum control effort, while ensuring the position never exceeds a certain limit.  
Originally introduced in [Bryson et al. 1963](https://doi.org/10.2514/3.2107), it is a standard test case for evaluating the ability of numerical solvers to handle state-constrained trajectories.

### Mathematical formulation

The problem can be stated as

```math
$$
\begin{aligned}
\min_{x,u} \quad & J(x,u) = \int_0^1 \frac{1}{2} u^2(t) \,\mathrm{d}t \\[1em]
\text{s.t.} \quad & \dot{x}_1(t) = x_2(t), \quad \dot{x}_2(t) = u(t), \\[0.5em]
& x_1(0) = 0, \; x_1(1) = 0, \; x_2(0) = 1, \; x_2(1) = -1, \\[0.5em]
& x_1(t) \le a.
\end{aligned}
$$
```

where $x_1$ represents position, $x_2$ velocity, $u$ acceleration (control), and $a$ is the maximum allowable displacement (e.g., $a=1/9$).

### Qualitative behaviour

The problem features a **second-order state constraint** because the control $u$ only appears in the second derivative of the constrained state $x_1$:

```math
$$
\ddot{x}_1(t) = u(t).
$$
```

When the trajectory is on a **boundary arc** ($x_1(t) = a$), the derivatives $\dot{x}_1$ and $\ddot{x}_1$ must be zero. This implies that while the constraint is active, the velocity $x_2$ is zero and the control $u$ is zero:

```math
$$
u(t) = 0.
$$
```

The solution structure changes based on the value of the constraint parameter $a$:

* **Unconstrained Case**: If $a$ is sufficiently large ($a \ge 1/4$), the state constraint is never active.
* **Touch Point**: For a critical value of $a$, the trajectory just touches the boundary $x_1 = a$ at a single point $t=0.5$.
* **Boundary Arc**: For smaller values (e.g., $a=1/9$), the trajectory remains on the boundary for a finite time interval. During this interval, the control vanishes.

### Characteristics

* Linear–quadratic dynamics with a second-order state inequality constraint.  
* Demonstrates a "bang-off-bang" control structure where the control is zero on the constraint boundary.  
* Used to test the accuracy of state constraint handling and the detection of switching times.

### References

* Bryson, A.E., Denham, W.F., & Dreyfus, S.E. (1963). *Optimal programming problems with inequality constraints I: necessary conditions for extremal solutions*.  
    AIAA Journal. [doi.org/10.2514/3.2107](https://doi.org/10.2514/3.2107)  
* Dymos Documentation: Bryson-Denham Problem. [dymos/examples/bryson_denham](https://openmdao.github.io/dymos/examples/bryson_denham/bryson_denham.html)