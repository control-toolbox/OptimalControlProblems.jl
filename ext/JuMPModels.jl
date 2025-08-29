module JuMPModels

using OptimalControlProblems
using JuMP
import CTModels: CTModels, time_grid, state, control, costate, iterations
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

Compute the discretised time grid for a given optimal control problem solved with JuMP.

# Arguments

- `problem::Symbol`: The name of the problem as defined in `metadata`.
- `model::JuMP.GenericModel`: The JuMP model containing the problem solution.

# Returns

- `t_jp::AbstractVector{Float64}`: A vector of time points spanning from initial time `t0 = 0` to the final time `tf`.

# Example

```julia-repl
julia> tgrid = OptimalControlProblems.time_grid(:my_problem, model)
0.0:0.1:1.0
```
"""
function OptimalControlProblems.time_grid(problem::Symbol, model::JuMP.GenericModel)

    # get N
    x_vars = metadata[problem][:state_name]
    x_jp_var = JuMP.value.(model[Symbol(x_vars[1])])
    N = length(x_jp_var) - 1

    ## time grid: we assume that t0 = 0
    time_data, time_value_or_index = metadata[problem][:final_time]

    t0 = 0
    tf = if time_data == :fixed
        time_value_or_index
    elseif time_data == :free
        v_vars = metadata[problem][:variable_name]
        value.(model[Symbol(v_vars[time_value_or_index])])
    else
        error("the final time must be :fixed or :free, not: ", time_data)
    end
    t_jp = range(t0, tf, N+1)

    return t_jp
end

"""
$(TYPEDSIGNATURES)

Extract and interpolate the state trajectory from a JuMP model of an optimal control problem.

# Arguments

- `problem::Symbol`: The name of the problem as defined in `metadata`.
- `model::JuMP.GenericModel`: The JuMP model containing the problem solution.

# Returns

- `fx::Function`: A function of continuous time returning the interpolated state.  
  If the state is scalar, the function returns a scalar; otherwise, a vector.

# Example

```julia-repl
julia> x = OptimalControlProblems.state(:my_problem, model)
julia> x(0.5)
[0.23, 0.71]
```
"""
function OptimalControlProblems.state(problem::Symbol, model::JuMP.GenericModel)

    # time grid
    T = CTModels.time_grid(problem, model)
    N = length(T) - 1

    # get dimension
    state_names = metadata[problem][:state_name]
    dim_x = length(state_names)

    # get state from the model
    X = zeros(N + 1, dim_x)
    for i in 1:dim_x
        x_name = state_names[i]
        X[:, i] = JuMP.value.(model[Symbol(x_name)])
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

- `problem::Symbol`: The name of the problem as defined in `metadata`.
- `model::JuMP.GenericModel`: The JuMP model containing the problem solution.

# Returns

- `fu::Function`: A function of continuous time returning the interpolated control.  
  If the control is scalar, the function returns a scalar; otherwise, a vector.

# Example

```julia-repl
julia> u = OptimalControlProblems.control(:my_problem, model)
julia> u(0.25)
0.42
```
"""
function OptimalControlProblems.control(problem::Symbol, model::JuMP.GenericModel)

    # time grid
    T = CTModels.time_grid(problem, model)
    N = length(T) - 1

    # get dimension
    control_names = metadata[problem][:control_name]
    dim_u = length(control_names)

    # get control from the model
    U = zeros(N + 1, dim_u)
    for i in 1:dim_u
        u_name = control_names[i]
        U[:, i] = JuMP.value.(model[Symbol(u_name)])
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

- `problem::Symbol`: The name of the problem as defined in `metadata`.
- `model::JuMP.GenericModel`: The JuMP model containing the problem solution.

# Returns

- `fp::Function`: A function of continuous time returning the interpolated costate.  
  If the costate is scalar, the function returns a scalar; otherwise, a vector.

# Example

```julia-repl
julia> p = OptimalControlProblems.costate(:my_problem, model)
julia> p(0.75)
[-0.12, 0.05]
```
"""
function OptimalControlProblems.costate(problem::Symbol, model::JuMP.GenericModel)

    # time grid
    T = CTModels.time_grid(problem, model)
    N = length(T) - 1

    # get dimension
    costate_names = metadata[problem][:costate_name]
    dim_x = length(costate_names)

    # get costate from the model
    P = zeros(N, dim_x)
    for i in 1:dim_x
        p_name = costate_names[i]
        P[:, i] = JuMP.dual.(model[Symbol(p_name)])
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

- `problem::Symbol`: The name of the problem as defined in `metadata`.
- `model::JuMP.GenericModel`: The JuMP model containing the problem solution.

# Returns

- `var::Union{Nothing,Float64,Vector{Float64}}`:  
  - `nothing` if the problem defines no additional variables.  
  - A scalar if there is one variable.  
  - A vector if multiple variables exist.

# Example

```julia-repl
julia> v = OptimalControlProblems.variable(:my_problem, model)
1.5
```
"""
function OptimalControlProblems.variable(problem::Symbol, model::JuMP.GenericModel)
    variable_names = metadata[problem][:variable_name]

    if isnothing(variable_names)
        return nothing
    end

    dim_v = length(variable_names)

    # get variable from the model
    v = zeros(dim_v)
    for i in 1:dim_v
        v_name = variable_names[i]
        v[i] = JuMP.value.(model[Symbol(v_name)])
    end

    # force scalar output when dimension is 1
    var = (dim_v == 1) ? v[1] : v

    return var
end

"""
$(TYPEDSIGNATURES)

Get the objective value from a JuMP model.

# Arguments

- `problem::Symbol`: The name of the problem as defined in `metadata`.
- `model::JuMP.GenericModel`: The JuMP model containing the problem solution.

# Example

```julia-repl
julia> OptimalControlProblems.objective(:my_problem, model)
1.5
```
"""
function OptimalControlProblems.objective(::Symbol, model::JuMP.GenericModel)
    return objective_value(model)
end

"""
$(TYPEDSIGNATURES)

Get the number of iterations from a JuMP model.

# Arguments

- `problem::Symbol`: The name of the problem as defined in `metadata`.
- `model::JuMP.GenericModel`: The JuMP model containing the problem solution.

# Example

```julia-repl
julia> OptimalControlProblems.iterations(:my_problem, model)
20
```
"""
function OptimalControlProblems.iterations(::Symbol, model::JuMP.GenericModel)
    return barrier_iterations(model)
end

end
