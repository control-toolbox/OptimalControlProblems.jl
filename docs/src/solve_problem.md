# Resolution of OptimalControlProblems.jl problems

We consider the **The Hanging Chain** Problem from [COPS package](https://www.mcs.anl.gov/~more/cops/) as an example. The problem is to find the shape of a chain hanging between two points a and b. The chain is assumed to be a uniform cable with a given length L. 
The aim is to find the shape of the chain that minimizes the potential energy. 

## Solving JuMP version

We need first to import the needed packages and the problem.

```@example main
using OptimalControlProblems
using JuMP

model = chain(JuMPBackend())
```

Then we can solve the problem using the `optimize!` function of `Ipopt` solver.	

```@example main
using Ipopt
# Set the optimizer
set_optimizer(model, Ipopt.Optimizer)
# Set the optimizer attributes
set_optimizer_attribute(model, "tol", 1e-8)
set_optimizer_attribute(model, "constr_viol_tol", 1e-6)
set_optimizer_attribute(model, "mu_strategy", "adaptive")
set_optimizer_attribute(model, "linear_solver", "mumps")
set_optimizer_attribute(model, "sb", "yes")
# Solve the model
optimize!(model)
```


## Solving OptimalControl version

We need first to import the needed packages and the problem.

```@example main2
using OptimalControlProblems
using CTDirect

_, model = chain(OptimalControlBackend())
```

Then we can solve the problem using the `ipopt` function of `NLPModelsIpopt`.	

```@example main2
using NLPModelsIpopt
# Solve the model
sol = NLPModelsIpopt.ipopt(
                model;
                # Optimizer attributes
                print_level=5,
                tol=1e-8,
                mu_strategy="adaptive",
                sb="yes",
                constr_viol_tol=1e-6,
            )
nothing # hide
```
