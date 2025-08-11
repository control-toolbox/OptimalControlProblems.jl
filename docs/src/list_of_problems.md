# [List of Problems](@id list-of-problems)

The problems are stored in [OptimalControlProblems.jl/ext](https://github.com/control-toolbox/OptimalControlProblems.jl/tree/main/ext) directory. JuMP models are stored in [JuMPModels](https://github.com/control-toolbox/OptimalControlProblems.jl/tree/main/ext/JuMPModels) directory and OptimalControl models are stored in [OptimalControlModels](https://github.com/control-toolbox/OptimalControlProblems.jl/tree/main/ext/OptimalControlModels).

For each problem, we provide the following data in [MetaData](https://github.com/control-toolbox/OptimalControlProblems.jl/tree/main/ext/MetaData) directory:

- `name::String`: problem name
- `nh::Int`: default number of discretization points
- `nvar::Int`: number of variables
- `ncon::Int`: number of general constraints
- `minimize::Bool`: true if optimize == minimize

To get the list of metadata, you can use the following code:

```@example metadata
using OptimalControlProblems
OptimalControlProblems.metadata
nothing # hide
```
To access the metadata of a specific problem, you can execute the following command:
```@example metadata
OptimalControlProblems.metadata[:chain]
```

The table below summarizes the names and status of the each problem:

<table>
<tr>
  <th rowspan="2">Problem</th>
  <th colspan="2">Convergence</th>
  <th colspan="2">Objective Value</th>
  <th colspan="3">Comparison</th>
  <th rowspan="2">Available Problems</th>
</tr>
<tr>
  <th>JuMP</th>
  <th>OptimalControl</th>
  <th>JuMP</th>
  <th>OptimalControl</th>
  <th>Init</th>
  <th>Objective</th>
  <th>||.||<sub>L<sup>2</sup></sub></th>
</tr>
<tr>
  <td>beam</td>
  <td>✅</td>
  <td>✅</td>
  <td>9.1213</td>
  <td>8.8986</td>
  <td>✅</td>
  <td>✅</td>
  <td>🟠</td>
  <td>🟠</td>
</tr>
<tr>
  <td>bioreactor</td>
  <td>✅</td>
  <td>✅</td>
  <td>19.053</td>
  <td>19.077</td>
  <td>✅</td>
  <td>✅</td>
  <td>🟠</td>
  <td>🟠</td>
</tr>
<tr>
  <td>cart_pendulum</td>
  <td>✅</td>
  <td>✅</td>
  <td>1.7439</td>
  <td>1.7444</td>
  <td>✅</td>
  <td>✅</td>
  <td>🟠</td>
  <td>🟠</td>
</tr>
<tr>
  <td>chain</td>
  <td>✅</td>
  <td>✅</td>
  <td>5.0698</td>
  <td>5.0698</td>
  <td>✅</td>
  <td>✅</td>
  <td>✅</td>
  <td>✅</td>
</tr>
<tr>
  <td>dielectrophoretic_particle</td>
  <td>✅</td>
  <td>✅</td>
  <td>7.8113</td>
  <td>7.8113</td>
  <td>✅</td>
  <td>✅</td>
  <td>✅</td>
  <td>✅</td>
</tr>
<tr>
  <td>double_oscillator</td>
  <td>✅</td>
  <td>✅</td>
  <td>0.0009</td>
  <td>0.0009</td>
  <td>✅</td>
  <td>✅</td>
  <td>🟠</td>
  <td>🟠</td>
</tr>
<tr>
  <td>ducted_fan</td>
  <td>✅</td>
  <td>✅</td>
  <td>1831.8</td>
  <td>1832.9</td>
  <td>✅</td>
  <td>✅</td>
  <td>🟠</td>
  <td>🟠</td>
</tr>
<tr>
  <td>electric_vehicle</td>
  <td>✅</td>
  <td>✅</td>
  <td>4.0246e5</td>
  <td>1.2287e6</td>
  <td>✅</td>
  <td>🟠</td>
  <td>🟠</td>
  <td>🟠</td>
</tr>
<tr>
  <td>glider</td>
  <td>✅</td>
  <td>✅</td>
  <td>1254.6</td>
  <td>1254.6</td>
  <td>✅</td>
  <td>✅</td>
  <td>✅</td>
  <td>✅</td>
</tr>
<tr>
  <td>insurance</td>
  <td>✅</td>
  <td>✅</td>
  <td>2.0879</td>
  <td>2.0578</td>
  <td>✅</td>
  <td>✅</td>
  <td>🟠</td>
  <td>🟠</td>
</tr>
<tr>
  <td>jackson</td>
  <td>✅</td>
  <td>✅</td>
  <td>0.1920</td>
  <td>0.1920</td>
  <td>✅</td>
  <td>✅</td>
  <td>✅</td>
  <td>✅</td>
</tr>
<tr>
  <td>moonlander</td>
  <td>🟠</td>
  <td>✅</td>
  <td>1.4148</td>
  <td>0.9621</td>
  <td>✅</td>
  <td>🟠</td>
  <td>🟠</td>
  <td>🟠</td>
</tr>
<tr>
  <td>quadrotor</td>
  <td>✅</td>
  <td>✅</td>
  <td>1.4928</td>
  <td>1.4925</td>
  <td>✅</td>
  <td>✅</td>
  <td>🟠</td>
  <td>🟠</td>
</tr>
<tr>
  <td>robbins</td>
  <td>✅</td>
  <td>✅</td>
  <td>20.628</td>
  <td>20.204</td>
  <td>✅</td>
  <td>✅</td>
  <td>🟠</td>
  <td>🟠</td>
</tr>
<tr>
  <td>robot</td>
  <td>✅</td>
  <td>✅</td>
  <td>9.1427</td>
  <td>9.1427</td>
  <td>🟠</td>
  <td>✅</td>
  <td>✅</td>
  <td>🟠</td>
</tr>
<tr>
  <td>rocket</td>
  <td>✅</td>
  <td>✅</td>
  <td>1.0128</td>
  <td>1.0128</td>
  <td>✅</td>
  <td>✅</td>
  <td>✅</td>
  <td>✅</td>
</tr>
<tr>
  <td>space_shuttle</td>
  <td>✅</td>
  <td>✅</td>
  <td>0.5959</td>
  <td>0.5959</td>
  <td>✅</td>
  <td>✅</td>
  <td>✅</td>
  <td>✅</td>
</tr>
<tr>
  <td>steering</td>
  <td>✅</td>
  <td>✅</td>
  <td>0.5546</td>
  <td>0.5546</td>
  <td>🟠</td>
  <td>✅</td>
  <td>✅</td>
  <td>🟠</td>
</tr>
<tr>
  <td>truck_trailer</td>
  <td>✅</td>
  <td>✅</td>
  <td>59.211</td>
  <td>59.259</td>
  <td>🟠</td>
  <td>✅</td>
  <td>🟠</td>
  <td>🟠</td>
</tr>
<tr>
  <td>vanderpol</td>
  <td>✅</td>
  <td>✅</td>
  <td>1.0480</td>
  <td>1.0480</td>
  <td>✅</td>
  <td>✅</td>
  <td>🟠</td>
  <td>🟠</td>
</tr>
</table>

**Legend**

The problems are solved with Ipopt and the parameters:
- Convergence
```julia
tol = 1e-8
constr_viol_tol = 1e-6
max_iter = 1000
mu_strategy = "adaptive"
linear_solver = "mumps"
max_wall_time = 500
sb = "yes"
```
- Comparison
```julia
ε_rel = 1e-1
ε_abs = 1e-8
p = 2
```
The symbols in the table means:

- Convergence
  - ✅ locally solved
  - 🟠 locally infeasible or maximum of iterations
  - ❌ error during execution
  
- Comparison
  - ✅ relative comparison is less than ```ε_rel```
  - 🟠 relative comparison is greater or equal than ```ε_rel```
  - ❌ error during execution

- Available Problems
  - ✅ current problem has no issue 
  - 🟠 current problem has at least one issue
  - ❌ error during execution