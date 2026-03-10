The **Mountain Car problem** is a classic benchmark in control and reinforcement learning.  
It involves an underpowered car that must climb a steep hill to reach a target position.  
Because the car's engine is too weak to climb the hill directly, it must first drive away from the goal to gain momentum by oscillating in a valley.  
The goal is to reach the target position in **minimum time**.

### Mathematical formulation

The problem can be stated as

```math
\begin{aligned}
\min_{x,v,u,t_f} \quad & J = t_f \\
	\text{s.t.} \quad & \dot{x}(t) = v(t), \\
& \dot{v}(t) = a \, u(t) - b \, \cos(c \, x(t)), \\
& x(0) = -0.5, \quad v(0) = 0.0, \\
& x(t_f) = 0.5, \quad v(t_f) \ge 0.0, \\
& -1.2 \le x(t) \le 0.5, \\
& -0.07 \le v(t) \le 0.07, \\
& -1 \le u(t) \le 1, \\
& t_f \ge 0.
\end{aligned}
```

### Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Power coefficient | $a$ | 0.001 |
| Gravity coefficient | $b$ | 0.0025 |
| Frequency coefficient | $c$ | 3.0 |
| Initial position | $x_0$ | -0.5 |
| Target position | $x_f$ | 0.5 |

### Qualitative behaviour

- The car must oscillate back and forth in the valley to build enough kinetic energy to climb the right hill.
- The optimal strategy typically involves a sequence of full-throttle accelerations in alternating directions (**bang-bang control**).
- The minimum time objective makes the problem sensitive to the initial guess for the final time.

### Characteristics

- Two-dimensional state space (position and velocity),
- Scalar control (effort/acceleration),
- **Free final time** with minimum time objective,
- State constraints (position and velocity bounds),
- Control constraints (bounds on acceleration).

### References

- **Dymos Documentation**. [*The Mountain Car Problem*](https://openmdao.github.io/dymos/examples/mountain_car/mountain_car.html).
  Provides a detailed description and implementation of the Mountain Car problem using the Dymos library.

- **OpenAI Gym**. [*MountainCar-v0*](https://gym.openai.com/envs/MountainCar-v0/).
  A standard reinforcement learning environment for the Mountain Car problem.

- Moore, A. W. (1990). *Efficient Memory-based Learning for Robot Control*. PhD thesis, University of Cambridge.  
  Introduces the Mountain Car problem as a benchmark for reinforcement learning and control.
