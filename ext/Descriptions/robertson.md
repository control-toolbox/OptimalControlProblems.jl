The **Robertson problem** is a classic benchmark in chemical kinetics, famously known for its extreme **stiffness**.  
It describes the concentration of three chemical species ($x, y, z$) undergoing a series of reactions with vastly different rate constants.  
While originally an initial value problem for ordinary differential equations, it is used here in an optimal control context to minimize the final concentration of one of the species.

### Mathematical formulation

The problem can be stated as

```math
\begin{aligned}
\min_{x,u} \quad & J(x,u) = -z(t_f) \[1em]
	ext{s.t.} \quad & \dot{x}(t) = -k_1 x(t) + k_2 y(t) z(t) \[0.5em]
& \dot{y}(t) = k_1 x(t) - k_2 y(t) z(t) - k_3 y(t)^2 \[0.5em]
& \dot{z}(t) = k_3 y(t)^2 \[0.5em]
& x(0) = 1, \; y(0) = 0, \; z(0) = 0, \[0.5em]
& -1 \le u(t) \le 1.
\end{aligned}
```

The rate constants are typically set to $k_1 = 0.04$, $k_2 = 3 \cdot 10^7$, and $k_3 = 10^4$, creating a system where reactions occur at very different timescales. Note that in this specific formulation, a dummy control $u$ is included within bounds, though it does not directly affect the dynamics.

### Qualitative behaviour

The Robertson problem is a prototypical example of a **stiff system**.  
Stiffness arises from the large disparity between the rate constants ($0.04$ vs $3 \cdot 10^7$).  
Even though the concentration of the intermediate species $y$ remains very small throughout the process, its high reactivity significantly constrains the step size of explicit numerical integrators.

In this optimal control version, the goal is to maximize the final concentration of the third species ($z(t_f)$), which is equivalent to minimizing $-z(t_f)$. The challenge for solvers lies in maintaining stability and accuracy over the long time interval ($t \in [0, 40]$) despite the extreme stiffness of the underlying dynamics.

### Characteristics

- Highly stiff system of nonlinear ordinary differential equations.  
- Three species with reaction rates differing by several orders of magnitude.  
- Serves as a rigorous test for the stability and efficiency of numerical integration schemes and optimization solvers.

### References

- Robertson, H. H. (1966). *The solution of a set of reaction rate equations*.  
  In J. Walsh (Ed.), Numerical Analysis: An Introduction (pp. 178–182). Academic Press.  
  The original source describing the chemical kinetics equations and highlighting the difficulties posed by their stiffness.

- Dymos Examples: Robertson Problem. [robertson_problem.html](https://openmdao.github.io/dymos/examples/robertson_problem/robertson_problem.html)  
  A modern implementation of the Robertson problem in the Dymos optimal control framework, used for benchmarking stiff ODE integration.
