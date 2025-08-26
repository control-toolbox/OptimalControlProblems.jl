The *insurance problem* is a benchmark in stochastic optimal control with applications in actuarial science and financial planning. It models the dynamic interplay between insurance coverage, medical expenses, health, revenue, and utility over time. Both the state trajectory $x(\cdot)$ and the control $u(\cdot)$ are decision variables. The aim is to maximise the expected utility subject to constraints on insurance, expenses, health, revenue, and marginal utility.

The problem can be written as

```math
\min_{x,\,u} J(x,u) = - \int_0^{t_f} U(t) \cdot f_x(t) \, dt
```

subject to the dynamics

```math
\dot{x}_1(t) = \bigl(1 - \gamma \, t \, v'(m(t)) / (dU/dR)(t)\bigr) \, h(t), \quad
\dot{x}_2(t) = h(t), \quad
\dot{x}_3(t) = (1 + \sigma) \, I(t) \, f_x(t),
```

with boundary conditions

```math
x_1(0) = 0, \quad x_2(0) = 0.001, \quad x_3(0) = 0, \qquad
x_3(t_f) = P,
```

and the constraints

```math
0 \le I(t) \le 1.5, \quad 0 \le m(t) \le 1.5, \quad 0 \le h(t) \le 25,
\quad 0 \le R(t), H(t), U(t), \quad 0.001 \le dU/dR(t),
```

where $f_x(t)$ is the illness distribution and $v(m) = m^{\alpha/2} / (1 + m^{\alpha/2})$.

### Qualitative behaviour

- Insurance $I(t)$ is dynamically adjusted to balance risk coverage and costs.  
- Medical expenses $m(t)$ interact with health $h(t)$ and influence utility.  
- Utility $U(t)$ and revenue $R(t)$ are coupled through marginal utility $dU/dR(t)$.  
- The system illustrates the trade-offs between health, wealth, and insurance over time.

### Characteristics

- Nonlinear dynamics linking health, expenses, and utility.  
- Multiple control variables with inequality constraints.  
- Stochastic component in the illness distribution $f_x(t)$.  
- Useful as a benchmark for actuarial optimal control problems.

### References

- Schmidli, H. (2014). *Stochastic Control for Insurance Companies*. Wiley.  
- Li, W. (2023). *Individual Insurance Choice: A Stochastic Control Approach*. University of Waterloo.  
- Guerdouh, D. (2022). *Optimal Control Strategies for the Premium Policy of an Insurance Firm*. MDPI.  
