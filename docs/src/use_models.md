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


!!! note "using OptimalControl"

    You can also use OptimalControl instead of CTDirect, as the former automatically imports the latter. This simplifies the code by requiring only one import statement.
    ```julia	
    # Instead of this
    using CTDirect
    # You can do this
    using OptimalControl  # This implicitly includes CTDirect
    ```


By running the following code, you can get the total number of `OptimalControl` models available in `OptimalControlProblems.jl`:

```@example main2
using CTDirect
using OptimalControlProblems
```

We can call for example the `cart_pendulum` problem as follows:

```@example main2
nlp = cart_pendulum(OptimalControlBackend())[2]
```

And we have also access to the DOCP information:

```@example main2
docp = cart_pendulum(OptimalControlBackend())[1]
```