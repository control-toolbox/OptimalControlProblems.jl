module JuMPModels

using OptimalControlProblems
using JuMP
import CTModels: 
    CTModels, 
    time_grid, 
    state, 
    control, 
    costate, 
    iterations, 
    control_components, 
    control_dimension,
    state_components,
    state_dimension,
    variable_components,
    variable_dimension
import ExaModels: ExaModels, variable, objective
using DocStringExtensions
using OrderedCollections: OrderedDict

# include problems files
rel_path_problems = "JuMPModels"
path = joinpath(dirname(@__FILE__), rel_path_problems)
files = filter(x -> x[(end - 2):end] == ".jl", readdir(path))
for file in files
    if file ≠ "JuMPModels.jl"
        include(joinpath(rel_path_problems, file))
    end
end

"""
$(TYPEDSIGNATURES)

Return the list of costate component names stored in a JuMP model.

# Arguments

- `model::JuMP.GenericModel`: A JuMP model that contains the key `:costate_components`.

# Returns

- `Vector{String}`: The names of the costate components.

# Example

```julia-repl
julia> costate_components(model)
["∂x", "∂v", "∂θ", "∂ω"]
```
"""
costate_components(model::JuMP.GenericModel) = model[:costate_components]


"""
$(TYPEDSIGNATURES)

Return the list of state component names stored in a JuMP model.

# Arguments

- `model::JuMP.GenericModel`: A JuMP model that contains the key `:state_components`.

# Returns

- `Vector{String}`: The names of the state components.

# Example

```julia-repl
julia> OptimalControlProblems.state_components(model)
["x", "v", "θ", "ω"]
```
"""
OptimalControlProblems.state_components(model::JuMP.GenericModel) = model[:state_components]


"""
$(TYPEDSIGNATURES)

Return the list of control component names stored in a JuMP model.

# Arguments

- `model::JuMP.GenericModel`: A JuMP model that contains the key `:control_components`.

# Returns

- `Vector{String}`: The names of the control components.

# Example

```julia-repl
julia> OptimalControlProblems.control_components(model)
["Fex"]
```
"""
OptimalControlProblems.control_components(model::JuMP.GenericModel) = model[:control_components]


"""
$(TYPEDSIGNATURES)

Return the list of additional variable component names stored in a JuMP model.

# Arguments

- `model::JuMP.GenericModel`: A JuMP model that contains the key `:variable_components`.

# Returns

- `Vector{String}`: The names of the additional variable components.

# Example

```julia-repl
julia> OptimalControlProblems.variable_components(model)
["tf", "ddx"]
```
"""
OptimalControlProblems.variable_components(model::JuMP.GenericModel) = model[:variable_components]


"""
$(TYPEDSIGNATURES)

Return the number of state components in a JuMP model.

# Arguments

- `model::JuMP.GenericModel`: A JuMP model that contains the key `:state_components`.

# Returns

- `Int`: The number of state components.

# Example

```julia-repl
julia> OptimalControlProblems.state_dimension(model)
4
```
"""
function OptimalControlProblems.state_dimension(model::JuMP.GenericModel) 
    return length(model[:state_components])
end


"""
$(TYPEDSIGNATURES)

Return the number of control components in a JuMP model.

# Arguments

- `model::JuMP.GenericModel`: A JuMP model that contains the key `:control_components`.

# Returns

- `Int`: The number of control components.

# Example

```julia-repl
julia> OptimalControlProblems.control_dimension(model)
1
```
"""
function OptimalControlProblems.control_dimension(model::JuMP.GenericModel) 
    return length(model[:control_components])
end


"""
$(TYPEDSIGNATURES)

Return the number of additional variable components in a JuMP model.

# Arguments

- `model::JuMP.GenericModel`: A JuMP model that contains the key `:variable_components`.

# Returns

- `Int`: The number of additional variable components.

# Example

```julia-repl
julia> OptimalControlProblems.variable_dimension(model)
2
```
"""
function OptimalControlProblems.variable_dimension(model::JuMP.GenericModel) 
    return length(model[:variable_components])
end

"""
$(TYPEDSIGNATURES)

Compute the discretised time grid for a given optimal control problem solved with JuMP.

# Arguments

- `model::JuMP.GenericModel`: The JuMP model containing the problem solution.

# Returns

- `t_jp::AbstractVector{Float64}`: A vector of time points spanning from initial time `t0 = 0` to the final time `tf`.

# Example

```julia-repl
julia> tgrid = OptimalControlProblems.time_grid(model)
0.0:0.1:1.0
```
"""
OptimalControlProblems.time_grid(model::JuMP.GenericModel) = model[:time_grid]()

"""
$(TYPEDSIGNATURES)

Extract and interpolate the state trajectory from a JuMP model of an optimal control problem.

# Arguments

- `model::JuMP.GenericModel`: The JuMP model containing the problem solution.

# Returns

- `fx::Function`: A function of continuous time returning the interpolated state.  
  If the state is scalar, the function returns a scalar; otherwise, a vector.

# Example

```julia-repl
julia> x = OptimalControlProblems.state(model)
julia> x(0.5)
[0.23, 0.71]
```
"""
function OptimalControlProblems.state(model::JuMP.GenericModel)

    # time grid
    T = CTModels.time_grid(model)
    N = length(T) - 1

    # get components and dimension
    x_vars = state_components(model)
    dim_x = state_dimension(model)

    # get state from the model
    X = zeros(N + 1, dim_x)
    for i in 1:dim_x
        X[:, i] = JuMP.value.(model[Symbol(x_vars[i])])
    end

    # interpolate
    N = size(X, 1)
    V = CTModels.matrix2vec(X[:, 1:dim_x], 1)
    x = CTModels.ctinterpolate(T[1:N], V)

    # force scalar output when dimension is 1
    fx = (dim_x == 1) ? deepcopy(t -> x(t)[1]) : deepcopy(t -> x(t))

    return fx
end

"""
$(TYPEDSIGNATURES)

Extract and interpolate the control trajectory from a JuMP model of an optimal control problem.

# Arguments

- `model::JuMP.GenericModel`: The JuMP model containing the problem solution.

# Returns

- `fu::Function`: A function of continuous time returning the interpolated control.  
  If the control is scalar, the function returns a scalar; otherwise, a vector.

# Example

```julia-repl
julia> u = OptimalControlProblems.control(model)
julia> u(0.25)
0.42
```
"""
function OptimalControlProblems.control(model::JuMP.GenericModel)

    # time grid
    T = CTModels.time_grid(model)
    N = length(T) - 1

    # get components and dimension
    u_vars = control_components(model)
    dim_u = control_dimension(model)

    # get control from the model
    U = zeros(N + 1, dim_u)
    for i in 1:dim_u
        U[:, i] = JuMP.value.(model[Symbol(u_vars[i])])
    end

    # interpolate
    M = size(U, 1)
    V = CTModels.matrix2vec(U[:, 1:dim_u], 1)
    u = CTModels.ctinterpolate(T[1:M], V)

    # force scalar output when dimension is 1
    fu = (dim_u == 1) ? deepcopy(t -> u(t)[1]) : deepcopy(t -> u(t))

    return fu
end

"""
$(TYPEDSIGNATURES)

Extract and interpolate the costate trajectory (dual variables associated with states) from a JuMP model of an optimal control problem.

# Arguments

- `model::JuMP.GenericModel`: The JuMP model containing the problem solution.

# Returns

- `fp::Function`: A function of continuous time returning the interpolated costate.  
  If the costate is scalar, the function returns a scalar; otherwise, a vector.

# Example

```julia-repl
julia> p = OptimalControlProblems.costate(model)
julia> p(0.75)
[-0.12, 0.05]
```
"""
function OptimalControlProblems.costate(model::JuMP.GenericModel)

    # time grid
    T = CTModels.time_grid( model)
    N = length(T) - 1

    # get dimension
    p_vars = costate_components(model)
    dim_x = state_dimension(model)

    # get costate from the model
    P = zeros(N, dim_x)
    for i in 1:dim_x
        P[:, i] = JuMP.dual.(model[Symbol(p_vars[i])])
    end

    # interpolate
    L = size(P, 1)
    V = CTModels.matrix2vec(P[:, 1:dim_x], 1)
    p = if length(T) == 2
        t -> P[1, 1:dim_x]
    else
        CTModels.ctinterpolate(T[1:L], V)
    end

    # force scalar output when dimension is 1
    fp = (dim_x == 1) ? deepcopy(t -> p(t)[1]) : deepcopy(t -> p(t))

    return fp
end

"""
$(TYPEDSIGNATURES)

Extract scalar or vector decision variables (such as final time when free) from a JuMP model of an optimal control problem.

# Arguments

- `model::JuMP.GenericModel`: The JuMP model containing the problem solution.

# Returns

- `var::Union{Float64,Vector{Float64}}`:  
  - `Float64[]` if the problem defines no additional variables.  
  - A scalar if there is one variable.  
  - A vector if multiple variables exist.

# Example

```julia-repl
julia> v = OptimalControlProblems.variable(model)
1.5
```
"""
function OptimalControlProblems.variable(model::JuMP.GenericModel)

    # get components and dimension
    v_vars = variable_components(model)
    dim_v = variable_dimension(model)
    if dim_v == 0
        return Float64[]
    end

    # get variable from the model
    v = zeros(dim_v)
    for i in 1:dim_v
        v[i] = JuMP.value.(model[Symbol(v_vars[i])])
    end

    # force scalar output when dimension is 1
    var = (dim_v == 1) ? v[1] : v

    return var
end

"""
$(TYPEDSIGNATURES)

Get the objective value from a JuMP model.

# Arguments

- `model::JuMP.GenericModel`: The JuMP model containing the problem solution.

# Example

```julia-repl
julia> OptimalControlProblems.objective(model)
1.5
```
"""
function OptimalControlProblems.objective(model::JuMP.GenericModel)
    return objective_value(model)
end

"""
$(TYPEDSIGNATURES)

Get the number of iterations from a JuMP model.

# Arguments

- `model::JuMP.GenericModel`: The JuMP model containing the problem solution.

# Example

```julia-repl
julia> OptimalControlProblems.iterations(model)
20
```
"""
function OptimalControlProblems.iterations(model::JuMP.GenericModel)
    return barrier_iterations(model)
end

end
