The **Jackson problem** is a classical benchmark in optimal control.  
It consists of controlling a three-dimensional system in which the first two states interact linearly under the effect of a single control input, while the third state accumulates based on the complementary control.  
The objective is to **minimise the third state at the final time**, while satisfying bounds on states and control, as well as initial and terminal conditions.  
The problem exhibits **singular arcs**, making it a useful benchmark for testing direct transcription and nonlinear programming methods.

---

### Mathematical formulation

The problem can be stated as

```math
\begin{aligned}
\min_{x_1, x_2, x_3, u} \quad & x_3(t_f) \\[0.5em]
\text{s.t.} \quad
& \dot{x}_1(t) = -u(t) (k_1 x_1(t) - k_2 x_2(t)), \\[0.25em]
& \dot{x}_2(t) = u(t) (k_1 x_1(t) - k_2 x_2(t)) - (1-u(t)) k_3 x_2(t), \\[0.25em]
& \dot{x}_3(t) = (1-u(t)) k_3 x_2(t), \\[0.5em]
& x(0) = [1, 0, 0], \quad
[0, 0, 0] \le x(t) \le [1.1, 1.1, 1.1], \\[0.25em]
& 0 \le u(t) \le 1, \quad t \in [0, t_f].
\end{aligned}
```

where $x_1, x_2, x_3$ are the state variables, $u$ is the control input, and $k_1, k_2, k_3$ are system parameters.

---

### System parameters

| Parameter | Symbol | Value | Description |
|-----------|--------|-------|-------------|
| Coupling coefficient | $k_1$ | 1 | Interaction between $x_1$ and $x_2$ |
| Coupling coefficient | $k_2$ | 10 | Interaction between $x_1$ and $x_2$ |
| Accumulation rate | $k_3$ | 1 | Growth of $x_3$ under complementary control |
| Final time | $t_f$ | fixed | Horizon of the control problem |
| Control bounds | $u$ | [0,1] | Single control input |
| State bounds | $x$ | $[0,1.1]^3$ | Box constraints for $x_1, x_2, x_3$ |

---

### Qualitative behaviour

- The first two states interact linearly under the effect of the control, while the third state accumulates according to $1-u(t)$.  
- The problem exhibits **singular arcs**, where the optimal control is neither at its lower nor upper bound but satisfies Hamiltonian conditions.  
- Optimal trajectories typically include **bang–bang segments** interleaved with singular arcs.  
- State and control constraints are respected at all times, and the final cost depends only on $x_3(t_f)$.

---

### Characteristics

- Linear–nonlinear three-dimensional dynamics with **one control input**.  
- State and control bounds.  
- Terminal cost depends on a single state.  
- Serves as a benchmark for testing solvers handling singular arcs and constrained optimal control.

---

### References

- **Jackson, E. A. (1968).** *The existence of singular extremals*. Journal of Optimization Theory and Applications.  
  Discusses the theoretical existence of singular extremals, forming the basis of the Jackson benchmark problem.

- **Biegler, L. T. (2010).** *Nonlinear Programming: Concepts, Algorithms, and Applications to Chemical Processes*. SIAM.  
  Provides background on nonlinear programming methods applicable to problems like Jackson.

- **BOCOP Repository: Jackson Example.** [https://github.com/control-toolbox/bocop/tree/main/bocop](https://github.com/control-toolbox/bocop/tree/main/bocop)  
  Contains the Jackson problem implementation for testing direct transcription and NLP solvers.
