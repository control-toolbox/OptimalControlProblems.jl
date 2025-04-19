# [Get a problem](@id get-problem)

For each problem in OptimalControlProblems, we need to use a specific backend to distinguish JuMP models from OptimalControl models.

## Using JuMP models

To use JuMP models from OptimalControlProblems, first install JuMP. Then, you can import the packages.

```@example main
using JuMP
using OptimalControlProblems
```

For instance, to get the JuMP model of the beam problem, execute:

```@example main
model = beam(JuMPBackend())
```

## Using OptimalControl models

To use OptimalControl models from OptimalControlProblems, first install OptimalControl. Then, you can import the packages.

```@example main2
using OptimalControl
using OptimalControlProblems
```

Now, to get the OptimalControl model of the beam problem, execute:

```@example main2
_, nlp = beam(OptimalControlBackend())
nlp # hide
```

And we have also access to the DOCP information:

```@example main2
docp, _ = beam(OptimalControlBackend())
docp # hide
```