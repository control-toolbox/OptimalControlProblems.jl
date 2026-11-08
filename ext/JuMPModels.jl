module JuMPModels

using OptimalControlProblems
using JuMP
using CTBase: CTBase
using CTModels: CTModels
using DocStringExtensions
using OrderedCollections: OrderedDict

# The schemes, written once for every problem. They live in a subdirectory: every file
# directly under `JuMPModels/` is a problem.
include(joinpath("JuMPModels", "schemes", "data.jl"))
include(joinpath("JuMPModels", "schemes", "tableaux.jl"))
include(joinpath("JuMPModels", "schemes", "build.jl"))

# Include only files belonging to this modeling. The file presence is the
# source of truth for availability.
rel_path_problems = "JuMPModels"
for problem in OptimalControlProblems.problems(JuMPBackend(); status=:all)
    include(joinpath(rel_path_problems, "$(problem).jl"))
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
CTModels.Models.state_components(model::JuMP.GenericModel) = model[:state_components]

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
function CTModels.Models.control_components(model::JuMP.GenericModel)
    return model[:control_components]
end

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
function CTModels.Models.variable_components(model::JuMP.GenericModel)
    return model[:variable_components]
end

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
function CTModels.Models.state_dimension(model::JuMP.GenericModel)
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
function CTModels.Models.control_dimension(model::JuMP.GenericModel)
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
function CTModels.Models.variable_dimension(model::JuMP.GenericModel)
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
CTModels.Components.time_grid(model::JuMP.GenericModel) = model[:time_grid]()

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
function CTModels.Components.state(model::JuMP.GenericModel)

    # time grid
    T = time_grid(model)
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
    V = CTBase.Core.matrix2vec(X[:, 1:dim_x], 1)
    x = CTBase.Interpolation.ctinterpolate(T[1:N], V)

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
function CTModels.Components.control(model::JuMP.GenericModel)

    # time grid and control at its nodes
    T = time_grid(model)
    dim_u = control_dimension(model)
    U = _control_at_nodes(model, length(T) - 1)

    # interpolate: linear between the nodes for a scheme with final control, steppost
    # (one control per step) otherwise
    V = CTBase.Core.matrix2vec(U[:, 1:dim_u], 1)
    u = if _control_interpolation(model) == :linear
        CTBase.Interpolation.ctinterpolate(T, V)
    else
        CTBase.Interpolation.ctinterpolate_constant(T, V)
    end

    # force scalar output when dimension is 1
    fu = (dim_u == 1) ? deepcopy(t -> u(t)[1]) : deepcopy(t -> u(t))

    return fu
end

# Sign convention of the multipliers (ct-conventions rules/signs.md, SGN1–SGN3): the
# multiplier `y` of a row `c(x) ∈ S` satisfies `∇F_min + Jᵀy = 0`, with `F_min` the
# objective of the equivalent minimization (`F` for a minimization, `-F` for a
# maximization). `JuMP.dual` returns the MOI `ConstraintDual`, a convention fixed by
# MathOptInterface, independent of the solver and of the objective sense — it is the one
# of the equivalent minimization, `∇F_min - Jᵀy_MOI = 0`. The multiplier is hence
# `-JuMP.dual`, for a `Min` and a `Max` model alike (SGN1: `min f` and `max -f` have the
# same multipliers).
# https://jump.dev/MathOptInterface.jl/stable/background/duality/
_multiplier(ref) = -JuMP.dual(ref)

"""
$(TYPEDSIGNATURES)

Extract and interpolate the costate trajectory (dual variables associated with states) from a JuMP model of an optimal control problem.

The sign convention is the one of `rules/signs.md` (ct-conventions, SGN1–SGN3): the
duals returned by the solver through `JuMP.dual` (the MOI `ConstraintDual`, independent
of the solver) are negated, see `_multiplier`.

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
function CTModels.Components.costate(model::JuMP.GenericModel)

    # time grid
    T = time_grid(model)
    N = length(T) - 1

    # get dimension
    p_vars = costate_components(model)
    dim_x = state_dimension(model)

    # get costate from the model
    P = zeros(N, dim_x)
    for i in 1:dim_x
        P[:, i] = _multiplier.(model[Symbol(p_vars[i])])
    end

    # interpolate
    L = size(P, 1)
    V = CTBase.Core.matrix2vec(P[:, 1:dim_x], 1)
    p = if length(T) == 2
        t -> P[1, 1:dim_x]
    else
        CTBase.Interpolation.ctinterpolate(T[1:L], V)
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
function CTModels.Components.variable(model::JuMP.GenericModel)

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
function CTModels.Components.objective(model::JuMP.GenericModel)
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
function CTModels.Solutions.iterations(model::JuMP.GenericModel)
    return barrier_iterations(model)
end

"""
$(TYPEDSIGNATURES)

Build the `CTModels.Solution` of a solved JuMP model, from the continuous model of `pb`, the
`Problem` of the same problem in OptimalControl. The states and the controls are given at the
nodes of the grid of the model, as CTDirect gives them: with one control per step
(`:euler`, `:euler_implicit`, `:midpoint`) the last control is repeated at the last node
and the control is piecewise constant; with the trapezoidal rule it is interpolated
linearly. The costate is given on the intervals of the grid, with the sign convention
of `rules/signs.md` (`-JuMP.dual`, see [`costate`](@ref)) — the same as an
OptimalControl solution (#326).

# Arguments

- `model::JuMP.GenericModel`: A solved JuMP model of the collection.
- `pb::OptimalControlProblems.Problem`: The problem of the same name, for the OptimalControl modelings.

# Throws

- `CTBase.Exceptions.PreconditionError`: If the model has not been solved.
- `CTBase.Exceptions.IncorrectArgument`: If the dimensions of the model and of `pb` differ.

# Example

```julia-repl
julia> using JuMP, Ipopt, OptimalControl, OptimalControlProblems
julia> model = beam(JuMPBackend(); grid_size=100)
julia> set_optimizer(model, Ipopt.Optimizer); optimize!(model)
julia> sol = solution(model, beam(OptimalControlBackend()))
julia> plot(sol)
```
"""
function OptimalControlProblems.solution(
    model::JuMP.GenericModel, pb::OptimalControlProblems.Problem
)
    JuMP.has_values(model) || throw(
        CTBase.Exceptions.PreconditionError(
            "The JuMP model has no solution";
            reason="the model has not been solved (termination status $(JuMP.termination_status(model)))",
            suggestion="call JuMP.optimize! on the model before building its solution",
            context="OptimalControlProblems.solution",
        ),
    )
    ocp = CTModels.Solutions.model(pb)
    dims = (
        state=(length(state_components(model)), CTModels.Models.state_dimension(ocp)),
        control=(length(control_components(model)), CTModels.Models.control_dimension(ocp)),
        variable=(
            length(variable_components(model)), CTModels.Models.variable_dimension(ocp)
        ),
    )
    for (name, (n_jump, n_ocp)) in pairs(dims)
        n_jump == n_ocp || throw(
            CTBase.Exceptions.IncorrectArgument(
                "The JuMP model and the problem differ in the dimension of the $name";
                got="$n_jump in the JuMP model",
                expected="$n_ocp, as in the problem $(pb.name)",
                suggestion="give the Problem of the same problem as the model",
                context="OptimalControlProblems.solution",
            ),
        )
    end

    T = collect(Float64, time_grid(model))
    N = length(T) - 1
    X = _values(model, state_components(model), JuMP.value)
    U = _control_at_nodes(model, N)
    P = _values(model, costate_components(model), _multiplier)
    v = Float64[JuMP.value(model[Symbol(c)]) for c in variable_components(model)]

    status = JuMP.termination_status(model)
    iterations = try
        Int(JuMP.barrier_iterations(model))
    catch err
        err isa JuMP.MOI.UnsupportedAttribute || rethrow()
        CTBase.Core.NotProvided
    end
    return CTModels.Solutions.build_solution(
        ocp,
        T,
        T,
        T[1:N],
        T,
        X,
        U,
        v,
        P;
        objective=JuMP.objective_value(model),
        iterations=iterations,
        message=JuMP.raw_status(model),
        status=Symbol(lowercase(string(status))),
        successful=status in (JuMP.MOI.OPTIMAL, JuMP.MOI.LOCALLY_SOLVED),
        control_interpolation=_control_interpolation(model),
    )
end

# The control at the `N+1` nodes, one column per component. A scheme without final control
# has one control per step: the last one is repeated at the last node, as CTDirect does.
function _control_at_nodes(model::JuMP.GenericModel, N::Int)
    U = _values(model, control_components(model), JuMP.value)
    return size(U, 1) == N ? vcat(U, U[end:end, :]) : U
end

# Interpolation of the control between the nodes: linear for a scheme with final control,
# steppost for one control per step.
function _control_interpolation(model::JuMP.GenericModel)
    return has_final_control(tableau(model[:scheme])) ? :linear : :constant
end

# Values of the variables or constraints `names` of the model, one column each.
function _values(model::JuMP.GenericModel, names, f)
    return reduce(
        hcat, [Float64[f(x) for x in collect(model[Symbol(name)])] for name in names]
    )
end

end
