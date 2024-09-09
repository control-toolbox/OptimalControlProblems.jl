# How to use models from OptimalControlProblems.jl

For each model in `OptimalControlProblems`, we need to use a specific bcakend to distinguish JuMP models from OtimalControl models.

## Using JuMP models

To use `JuMP` models from `OptimalControlProblems.jl`, you need to have `JuMP` installed. 

By running the following code, you can get the total number of `JuMP` models available in `OptimalControlProblems.jl`:

```@example main
using JuMP
using OptimalControlProblems
```

We can call for example the `cart_pendulum` problem as follows:

```@example main
model = cart_pendulum(JuMPBackend())
```

## Using OptimalControl models

To use `OptimalControl` models from `OptimalControlProblems.jl`, you need to have `CTDirect` installed.

By running the following code, you can get the total number of `OptimalControl` models available in `OptimalControlProblems.jl`:

```@example main2
using CTDirect
using OptimalControlProblems
```

We can call for example the `cart_pendulum` problem as follows:

```@example main2
model = cart_pendulum(OptimalControlBackend())
```
