This problem involves swinging up a pendulum mounted on a cart, a classical **underactuated system**.  
The goal is to move the pendulum from its downward equilibrium to the upright position while controlling the horizontal motion of the cart, in **minimum time**.

### System Dynamics

The system has four states and one control:

- $x$ : cart position  
- $v$ : cart velocity  
- $\theta$ : pendulum angle from downward vertical  
- $\omega$ : pendulum angular velocity  
- $F_{\rm ex}$ : horizontal force applied to the cart (control)  

The dynamics are expressed as

```math
\dot{x} = v
```

```math
\dot{v} = -\frac{1}{J} \, c
```

```math
\dot{\theta} = \omega
```

```math
\dot{\omega} = \alpha(\dot{v})
```

where

```math
\alpha(ddx) = \frac{0.5 \, L \, m}{I + 0.25 \, m L^2} \big(-ddx \cos\theta - g \sin\theta\big)
```

```math
\text{ddCOG} = L \, \omega \, [-\sin\theta, \cos\theta] + \frac{L}{2} \,[\cos\theta, \sin\theta] \, \alpha(ddx) + [ddx,0]
```

```math
\text{FXFY} = m \, \text{ddCOG} + [0, m g]
```

```math
c = -\text{FXFY}_x + F_{\rm ex} - m_{\rm cart} ddx - J ddx
```

Here, $J$ represents the effective mass of the cart, $m$ and $m_{\rm cart}$ are the pendulum and cart masses, $L$ is the pendulum length, and $I$ its moment of inertia. The variable $ddx$ is an auxiliary variable related to the cart acceleration used for the dynamics formulation.

### Boundary Conditions

- **Initial conditions**:

```math
x(0) = 0, \quad \theta(0) = 0, \quad \omega(0) = 0
```

- **Final conditions**:

```math
\theta(T) = \pi, \quad \omega(T) = 0
```

- Cart position and velocity constraints:

```math
|x(t)| \le 1, \quad |v(t)| \le 2
```

- Control limits:

```math
|F_{\rm ex}(t)| \le 5
```

- Time horizon:

```math
T \ge 0.1
```

### Objective

The goal is to **minimize the final time** $T$:

```math
J = T \to \min
```

subject to the dynamics, boundary conditions, and state/control constraints.

### References

- Vanroye, L., Sathya, A., De Schutter, J., & Decré, W. (2023). FATROP: A Fast Constrained Optimal Control Problem Solver for Robot Trajectory Optimization and Control. *arXiv preprint arXiv:2303.16746*. Retrieved from https://arxiv.org/pdf/2303.16746  
- Åström, K. J., & Furuta, K. (2000). Swinging up a pendulum by energy control. *Automatica*, 36(2), 287–295.  
- Lipp, T., & Boyd, S. (2014). Variations and extensions of the cart-pole swing-up problem. *Stanford University Technical Report*.  
