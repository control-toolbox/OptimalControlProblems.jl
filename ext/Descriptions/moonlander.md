The **Moonlander optimal control problem** is a benchmark in constrained optimal control.  
It models the powered descent of a spacecraft aiming to land on a specified target on the Moon's surface in minimum time.  
The system comprises six state variables: the horizontal position $p_1(t)$, vertical position $p_2(t)$, horizontal velocity $dp_1(t)$, vertical velocity $dp_2(t)$, orientation angle $\theta(t)$, and angular velocity $d\theta(t)$.  
The control variables $F_1(t)$ and $F_2(t)$ represent the thrust forces applied by the lander's engines.  
The objective is to **minimise the final time** $t_f$ while ensuring a soft landing at the target, subject to control and physical constraints.

### Mathematical formulation

The problem can be stated as

```math
\begin{aligned}
\min_{p_1,p_2,dp_1,dp_2,\theta,d\theta,F_1,F_2,t_f} \quad & t_f \\
\text{s.t.} \quad
\dot{x}(t) &= \begin{bmatrix} dp_1(t) \\ dp_2(t) \\ \ddot{p}_1(t) \\ \ddot{p}_2(t) \\ d\theta(t) \\ \ddot{\theta}(t) \end{bmatrix} = \begin{bmatrix} dp_1(t) \\ dp_2(t) \\ \frac{1}{m} F_{\text{tot},1}(t) \\ \frac{1}{m} F_{\text{tot},2}(t) - g \\ \frac{1}{I} (D/2) (F_2(t) - F_1(t)) \end{bmatrix}, \\
0 \le F_1(t), F_2(t) &\le 2g, \quad 0.1 \le t_f \le 1.0, \\
p_1(0) = p_2(0) = dp_1(0) = dp_2(0) = \theta(0) = d\theta(0) &= 0, \\
p_1(t_f) = 5.0, \; p_2(t_f) = 5.0, \; dp_1(t_f) = dp_2(t_f) &= 0.
\end{aligned}
```

Here, $x(t) = (p_1, p_2, dp_1, dp_2, \theta, d\theta)$, and the total thrust in the lander frame $F_{\text{tot}}$ is obtained by rotating the engine thrusts according to the lander's orientation.

### Parameter values

- Mass: $m = 1$  
- Lunar gravity: $g = 9.81$  
- Moment of inertia: $I = 0.1$  
- Distance between thrusters: $D = 1$  
- Maximum thrust: $F_{\max} = 2g$  
- Target position: $p_{\rm target} = [5.0, 5.0]$  
- Final time bounds: $t_f \in [0.1, 1.0]$  

### Qualitative behaviour

The optimal solution typically exhibits a **bang–bang structure**, where thrusts switch between minimum and maximum values, ensuring minimal landing time while satisfying position and velocity constraints.  
Orientation dynamics $\theta(t)$ and $d\theta(t)$ play a critical role in directing the thrust to achieve the target.

### Characteristics

- Nonlinear coupled dynamics with orientation-dependent thrust,  
- Control constraints and bounded final time ensuring feasibility,  
- Terminal position and velocity constraints for a soft landing,  
- Benchmark problem for testing direct transcription and NLP solvers.

### References

- Gazzola, F., & Marchini, E. M. (2021). *The moon lander optimal control problem revisited*. Mathematics in Engineering, 3(5), 1–14. [doi:10.3934/mine.2021040](https://doi.org/10.3934/mine.2021040)  
  Provides a detailed analysis of the Moonlander problem, including minimum-time trajectories and safe landing curves.

- GPOPS-II Examples: Moon Lander Problem. [GPOPS-II Moon Lander Example](https://www.gpops2.com/Examples/MoonLander.html)  
  Illustrates practical implementation of the Moonlander OCP in a direct transcription framework, useful for benchmarking.

- Vanroye, L., Sathya, A., De Schutter, J., & Decré, W. (2023). *FATROP: A Fast Constrained Optimal Control Problem Solver*. *arXiv preprint arXiv:2303.16746*.  
  Discusses solver strategies applied to constrained trajectory optimization problems, including applications relevant to the Moonlander.
