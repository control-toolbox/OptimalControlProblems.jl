module JuMPModels

using OptimalControlProblems
using JuMP
import CTModels: CTModels, time_grid, state, control, costate

rel_path_problems = "JuMPModels"
path = joinpath(dirname(@__FILE__), rel_path_problems)

files = filter(x -> x[(end - 2):end] == ".jl", readdir(path))
for file in files
    if file ≠ "JuMPModels.jl"
        include(joinpath(rel_path_problems, file))
    end
end

#
function CTModels.time_grid(problem::Symbol, model::JuMP.GenericModel)

    # get N
    x_vars = OptimalControlProblems.metadata[problem][:state_name]
    x_jp_var = JuMP.value.(model[Symbol(x_vars[1])])
    N = length(x_jp_var) - 1

    ## time grid: we assume that t0 = 0
    time_data, time_var_name, time_value = OptimalControlProblems.metadata[problem][:time]

    t0 = 0
    t_jp = if time_data == "final_time"
        if time_value !== nothing
            tf = time_value
        else
            tf = value.(model[Symbol(time_var_name)])
        end
        range(t0, tf, N+1)
    elseif time_data == "step"
        if time_value !== nothing
            h = time_value
            tf = h * N
            range(t0, tf, N+1)
        else
            h = value.(model[Symbol(time_var_name)])
            if isa(h, Number)
                tf = h * N
                range(t0, tf, N+1)
            else
                cumsum([0, h...])
            end
        end
    end
    return t_jp

end

# todo: function of time
# todo: return a scalar if of dimension 1
# todo: add variable getter?!
function CTModels.state(problem::Symbol, model::JuMP.GenericModel)

    # time grid
    T = CTModels.time_grid(problem, model)
    N = length(T) - 1

    # get dimension
    state_names = OptimalControlProblems.metadata[problem][:state_name]
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

function CTModels.control(problem::Symbol, model::JuMP.GenericModel)

    # time grid
    T = CTModels.time_grid(problem, model)
    N = length(T) - 1

    # get dimension
    control_names = OptimalControlProblems.metadata[problem][:control_name]
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

function CTModels.costate(problem::Symbol, model::JuMP.GenericModel)

    # time grid
    T = CTModels.time_grid(problem, model)
    N = length(T) - 1

    # get dimension
    costate_names = OptimalControlProblems.metadata[problem][:costate_name]
    dim_x = length(costate_names)

    # get state from the model
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


end
