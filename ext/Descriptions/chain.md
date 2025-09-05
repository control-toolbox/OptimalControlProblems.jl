The **Hanging Chain problem** is a classical benchmark in optimal control.  
It consists of moving a chain from a given initial horizontal position to a target horizontal position while controlling the horizontal velocity of the chain.  
The objective is to reach the final configuration in a way that **minimises the vertical displacement** $x_2$ of the chain.  
This problem is widely used to test trajectory optimisation and direct transcription methods for nonlinear optimal control.

### Mathematical formulation

The problem can be stated as

```math
\begin{aligned}
\min_{x_1, x_2, x_3, u} \quad & x_2(T) \\[0.5em]
\text{s.t.} \quad &
\dot{x}_1 = u, \quad
\dot{x}_2 = x_1 \sqrt{1 + u^2}, \quad
\dot{x}_3 = \sqrt{1 + u^2}, \\[0.5em]
& x_1(0) = a, \quad x_2(0) = 0, \quad x_3(0) = 0, \\[0.25em]
& x_1(T) = b, \quad x_3(T) = L.
\end{aligned}
```

### System parameters

| Parameter | Symbol | Value | Description |
|-----------|--------|-------|-------------|
| Horizontal start | $a$ | 1 | Initial $x_1$ position |
| Horizontal end | $b$ | 3 | Final $x_1$ position |
| Chain length | $L$ | 4 | Total length of the chain |
| Final time | $T$ | 1 | Duration of the motion |
| Control input | $u$ | — | Horizontal velocity of the chain |

### Qualitative behaviour

The optimal control trajectory exploits the nonlinear coupling between horizontal and vertical motion:  
- The state $x_1$ directly follows the control input $u$ (horizontal velocity).  
- The state $x_2$ evolves depending on $x_1$, which introduces nonlinear dynamics in the vertical motion.  
- The state $x_3$ measures the chain extension and grows with the magnitude of $u$.

The control typically balances horizontal movement to minimise the vertical displacement at the final time.

### Characteristics

- Nonlinear dynamics with three states and one control.  
- Minimum vertical displacement objective with boundary constraints.  
- Serves as a benchmark for trajectory optimisation and direct transcription methods in nonlinear optimal control.

### References

- **More, J. J., & Munson, T. S. (2000).** *The Hanging Chain Problem as an Optimal Control Problem*.  
  [mcs.anl.gov/~more/cops/bcops/chain.html](https://www.mcs.anl.gov/~more/cops/bcops/chain.html)  
  This benchmark explicitly formulates the Hanging Chain (catenary) as an optimal control problem, including direct transcription to an NLP. It is widely used as a test case for solver performance in AMPL, MINOS, and other tools.

- **Dolan, E. D., & More, J. J. (2001).** *Benchmarking Optimisation Software with COPS 3.0*. Technical Report ANL/MCS-TM-246, Argonne National Laboratory.  
  [mcs.anl.gov/~more/cops](https://www.mcs.anl.gov/~more/cops)  
  The COPS benchmark collection officially includes the Hanging Chain problem as one of its core examples. It provides comprehensive problem formulation, discretisation strategies, and solver comparison results.

- **Rutquist, P. E., & Edvall, M. M. (2009).** *Hanging Chain problem example in PROPT MATLAB Optimal Control Software*.  
  Included in the PSOPT distribution’s example suite, as credited by PSOPT’s list of examples ([psopt.net](https://www.psopt.net/list-of-examples)). This demonstrates a practical implementation of the Hanging Chain problem via direct transcription using MATLAB-based optimal control software.

- **Huygens, C. (1690).** *Horologium Oscillatorium*. Paris: F. Muguet.  
  This foundational work contains one of the earliest studies of the hanging chain (catenary) curve. Huygens, along with Leibniz and Johann Bernoulli, contributed to the historical derivation of the catenary equation, which underlies modern optimal control formulations of the problem.
