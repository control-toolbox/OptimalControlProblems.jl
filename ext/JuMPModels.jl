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
function CTModels.state(problem::Symbol, model::JuMP.GenericModel)
    x_vars = OptimalControlProblems.metadata[problem][:state_name]
    x_jp_vars = [JuMP.value.(model[Symbol(xv)]) for xv in x_vars]
    inds_x = axes(x_jp_vars[1], 1)
    x_jp = [[x_jp_vars[j][i] for j in 1:length(x_vars)] for i in inds_x]
    return x_jp
end

function CTModels.costate(problem::Symbol, model::JuMP.GenericModel)
    p_vars = OptimalControlProblems.metadata[problem][:costate_name]
    p_jp_vars = [JuMP.dual.(model[Symbol(pv)]) for pv in p_vars]
    inds_p = axes(p_jp_vars[1], 1)
    p_jp = -[[p_jp_vars[j][i] for j in 1:length(p_vars)] for i in inds_p]
    push!(p_jp, p_jp[end]) # we add one element
    return p_jp
end

function CTModels.control(problem::Symbol, model::JuMP.GenericModel)
    u_vars = OptimalControlProblems.metadata[problem][:control_name]
    u_jp_vars = [JuMP.value.(model[Symbol(uv)]) for uv in u_vars]
    inds_u = axes(u_jp_vars[1], 1)
    u_jp = [[u_jp_vars[j][i] for j in 1:length(u_vars)] for i in inds_u]
    return u_jp
end

end
