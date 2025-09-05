This problem models a simple system of chemical reactions introduced in Jackson (1968) and later discussed by Biegler (2010):

```math
A \;\overset{1}{\rightleftharpoons}\; B \;\overset{2}{\longrightarrow}\; C.
```

The first reaction is reversible ($A \leftrightarrow B$), while the second one is one-sided ($B \to C$).  
The control variable $u(t)\in [0,1]$ represents the fraction of catalyst allocated between the two reactions:

- for $u=1$, the catalyst favours the reversible $A \leftrightarrow B$ pathway,  
- for $u=0$, the catalyst favours the irreversible $B \to C$ pathway.  

The aim is to **maximise the production of $C$** at a fixed terminal time $T$.

### Problem formulation

Let $a(t), b(t), c(t)$ denote the mole fractions of $A, B, C$ respectively, and $k_1, k_2, k_3$ the kinetic constants.  
The optimal control problem is

```math
\begin{aligned}
\max_{u(\cdot)} \quad & c(T) \\
\dot a(t) &= - u(t)\,(k_1 a(t) - k_2 b(t)), \\
\dot b(t) &= \; u(t)\,(k_1 a(t) - k_2 b(t)) - (1-u(t))\,k_3 b(t), \\
\dot c(t) &= (1-u(t))\,k_3 b(t), \\
u(t) &\in [0,1], \\
a(0) &= 1, \quad b(0)=c(0)=0.
\end{aligned}
```

The state constraints are $a,b,c \geq 0$, and by invariance one has $a(t)+b(t)+c(t)=1$.

### Qualitative behaviour

The Hamiltonian is linear in the control, so the optimal solution typically consists of **bang–bang arcs** and possibly **singular arcs**.  

For the parameters $k_1=k_3=1$, $k_2=10$, and $T=4$, the optimal control exhibits a **bang–singular–bang** structure (1 → singular → 0).  
The singular arc corresponds to an intermediate phase where the catalyst is shared between the two reactions.

### Parameter identification (optional extension)

The same model can be extended to parameter estimation: given experimental observations of $a(t), b(t), c(t)$ under a known control input $u(t)$, the unknown kinetic constants $k_1, k_2, k_3$ can be identified using a least-squares fit.  
Simulated data with added noise reproduce the original parameters with high accuracy (see Table below).

| Parameter        | True value | Identified value |
|------------------|------------|------------------|
| $k_1$            | 1          | 0.998            |
| $k_2$            | 10         | 9.97             |
| $k_3$            | 1          | 1.001            |

### References

- Jackson, E. A. (1968). *The existence of singular extremals*. Journal of Optimization Theory and Applications.  
- Biegler, L. T. (2010). *Nonlinear Programming: Concepts, Algorithms, and Applications to Chemical Processes*. SIAM.  
- BOCOP repository: https://github.com/control-toolbox/bocop
