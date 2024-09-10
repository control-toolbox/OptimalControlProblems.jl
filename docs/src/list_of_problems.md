# List of Problems

## Where to find the problems
The problems are stored in [OptimalControlProblems.jl/ext](https://github.com/control-toolbox/OptimalControlProblems.jl/tree/main/ext) directory. `JuMP` models are stored in [JuMPModels](https://github.com/control-toolbox/OptimalControlProblems.jl/tree/main/ext/JuMPModels) directory and `OptimalControl` models are stored in [OptimalControlModels](https://github.com/control-toolbox/OptimalControlProblems.jl/tree/main/ext/OptimalControlModels).

For each problem, we provide the following data in [MetaData](https://github.com/control-toolbox/OptimalControlProblems.jl/tree/main/ext/MetaData) directory:
- `name::String`: problem name
- `nh::Int`: default number of discretization points
- `nvar::Int`: number of variables
- `ncon::Int`: number of general constraints
- `minimize::Bool`: true if optimize == minimize

To get the list of metadata, you can use the following code:
```julia
using OptimalControlProblems
OptimalControlProblems.metadata
```

## List of Problems
The table below summarizes the status of the each problem:

| Problem | With JuMP | With OptimalControl |
| --- | --- | --- | 
| `chain` |   ✅  |   ✅|
| `glider` |  ✅  |  ✅ |
| `robot` |  ✅ | ✅|
| `rocket` |  ✅ | ✅|
| `steering` |  ✅ | ✅|
| `space_shuttle` |  ✅ |  ❌|
| `cart_pendulum` | ✅ | ✅|
| `moonlander` | ✅ | ❌|
| `truck_trailer` | ❌ | ❌|
| `quadrotor` | ✅ | ❌|
| `dielectrophoretic_particle` | ✅ | ✅| 
| `ducted_fan` | ✅ | ✅| 
| `double_oscillator` | ✅ | ✅|
| `electrical_vehicle` | ✅ | ✅|
| `vanderpol` | ✅ | ✅|

