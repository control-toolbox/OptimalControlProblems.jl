# Comparison of the modelings of a problem (JuMP, OptimalControl, OptimalControlScalarized).
#
# Used by `suite/comparison/` (small grid, quick) and `cron/comparison/` (default grid).
# Numbers only: no plot. The visual comparison is left to the cron report (#310).
module Comparison

using Test: Test
using CTDirect: CTDirect
using Ipopt: Ipopt
using JuMP: JuMP
using MadNLP: MadNLP
using MadNLPMumps: MadNLPMumps
using NLPModelsIpopt: NLPModelsIpopt
using OptimalControl: OptimalControl # loads the OptimalControlModels extension
using OptimalControlProblems: OptimalControlProblems

const OCP = OptimalControlProblems

# Tolerances (relative, absolute), as in the former `comparison`.
const GRID = (rtol=1e-2, atol=1e-6)
const OBJECTIVE = (rtol=1e-4, atol=1e-6)
const STATE = (rtol=1e-1, atol=1e-6)
const CONTROL = (rtol=1e-1, atol=1e-6)
const VARIABLE = (rtol=1e-1, atol=1e-6)

"""Modelings compared, in the order of `reference` first."""
const MODELINGS = (:JuMP, :OptimalControl, :OptimalControlScalarized)
const REFERENCE = :JuMP

"""Problems available in the three modelings (valid status), derived from the catalogue."""
function problems()
    return OCP.problems(
        OCP.JuMPBackend(),
        OCP.OptimalControlBackend(),
        OCP.OptimalControlScalarizedBackend(),
    )
end

"""L² norm of a one-dimensional signal `X` on the time grid `T` (trapezoidal rule)."""
function L2_norm(T, X)
    s = 0.0
    for i in 1:(length(T) - 1)
        s += 0.5 * (X[i]^2 + X[i + 1]^2) * (T[i + 1] - T[i])
    end
    return sqrt(s)
end

"""Sampled solution of one modeling."""
struct Data
    t::Vector{Float64}
    x::Vector{Vector{Float64}}
    u::Vector{Vector{Float64}}
    v::Vector{Float64}
    objective::Float64
    iterations::Int
    nvar::Int
    ncon::Int
    xnames::Vector{String}
    unames::Vector{String}
    vnames::Vector{String}
end

vec1(a::Number) = [Float64(a)]
vec1(a) = Float64.(a)

function from_solution(sol, ocp, nlp)
    t = collect(Float64, OCP.time_grid(sol))
    return Data(
        t,
        [vec1(OCP.state(sol)(s)) for s in t],
        [vec1(OCP.control(sol)(s)) for s in t],
        vec1(OCP.variable(sol)),
        Float64(OCP.objective(sol)),
        Int(OCP.iterations(sol)),
        nlp.meta.nvar,
        nlp.meta.ncon,
        String.(OCP.state_components(ocp)),
        String.(OCP.control_components(ocp)),
        String.(OCP.variable_components(ocp)),
    )
end

"""Solve `problem` with JuMP and Ipopt, at most `max_iter` iterations."""
function solve_jump(problem::Symbol; grid_size::Int, max_iter::Int)
    o = Main.TestSolverOptions
    model = getfield(OCP, problem)(OCP.JuMPBackend(); grid_size=grid_size)
    JuMP.set_optimizer(model, Ipopt.Optimizer)
    JuMP.set_silent(model)
    JuMP.set_optimizer_attribute(model, "tol", o.TOL)
    JuMP.set_optimizer_attribute(model, "max_iter", max_iter)
    JuMP.set_optimizer_attribute(model, "mu_strategy", o.MU_STRATEGY)
    JuMP.set_optimizer_attribute(model, "linear_solver", "mumps")
    JuMP.set_optimizer_attribute(model, "max_wall_time", o.MAX_WALL_TIME)
    JuMP.set_optimizer_attribute(model, "sb", o.SB)
    JuMP.optimize!(model)
    t = collect(Float64, OCP.time_grid(model))
    return Data(
        t,
        [vec1(OCP.state(model)(s)) for s in t],
        [vec1(OCP.control(model)(s)) for s in t],
        vec1(OCP.variable(model)),
        Float64(OCP.objective(model)),
        Int(OCP.iterations(model)),
        JuMP.num_variables(model),
        JuMP.num_constraints(model; count_variable_in_set_constraints=false),
        String.(OCP.state_components(model)),
        String.(OCP.control_components(model)),
        String.(OCP.variable_components(model)),
    )
end

"""Solve `problem` with the OptimalControl modeling and Ipopt."""
function solve_optimalcontrol(problem::Symbol; grid_size::Int, max_iter::Int)
    o = Main.TestSolverOptions
    docp = getfield(OCP, problem)(OCP.OptimalControlBackend(); grid_size=grid_size)
    nlp = CTDirect.nlp_model(docp)
    stats = NLPModelsIpopt.ipopt(
        nlp;
        print_level=0,
        tol=o.TOL,
        mu_strategy=o.MU_STRATEGY,
        sb=o.SB,
        max_iter=max_iter,
        max_wall_time=o.MAX_WALL_TIME,
    )
    sol = CTDirect.build_OCP_solution(docp, stats)
    return from_solution(sol, CTDirect.ocp_model(docp), nlp)
end

"""Solve `problem` with the scalarized OptimalControl modeling and MadNLP."""
function solve_scalarized(problem::Symbol; grid_size::Int, max_iter::Int)
    o = Main.TestSolverOptions
    docp = getfield(OCP, problem)(
        OCP.OptimalControlScalarizedBackend(), :madnlp, :exa; grid_size=grid_size
    )
    nlp = CTDirect.nlp_model(docp)
    stats = MadNLP.madnlp(
        nlp;
        print_level=MadNLP.ERROR,
        tol=o.TOL,
        max_iter=max_iter,
        max_wall_time=o.MAX_WALL_TIME,
        linear_solver=MadNLPMumps.MumpsSolver,
    )
    sol = CTDirect.build_OCP_solution(docp, stats)
    return from_solution(sol, CTDirect.ocp_model(docp), nlp)
end

const SOLVERS = (
    JuMP=solve_jump,
    OptimalControl=solve_optimalcontrol,
    OptimalControlScalarized=solve_scalarized,
)

"""Solve `problem` with every modeling; returns a `NamedTuple` of `Data`."""
function solve_all(problem::Symbol; grid_size::Int, max_iter::Int)
    return map(f -> f(problem; grid_size=grid_size, max_iter=max_iter), SOLVERS)
end

# Broken (not failed) when a comparison is not met: the status-aware loop of #303/#304 will
# turn the broken ones of `:valid` problems into failures. Same as the former `@my_test_broken`.
check(ok::Bool) = Main.TestHelpers.@broken_if_false ok

within(a, b; rtol, atol) = abs(a - b) < max(0.5 * (abs(a) + abs(b)) * rtol, atol)

function within_L2(t, a, b; rtol, atol)
    d = L2_norm(t, a .- b)
    return d < max(0.5 * (L2_norm(t, a) + L2_norm(t, b)) * rtol, atol)
end

"""Number of variables and constraints, names of the components."""
function compare_structure(ref::Data, other::Data)
    check(ref.xnames == other.xnames)
    check(ref.unames == other.unames)
    check(ref.vnames == other.vnames)
    check(ref.nvar == other.nvar)
    check(ref.ncon == other.ncon)
    return nothing
end

"""Time grids; returns whether they are comparable (same length, same final time)."""
function compare_grid(ref::Data, other::Data)
    same_length = check(length(ref.t) == length(other.t))
    same_final = check(within(ref.t[end], other.t[end]; GRID...))
    comparable = same_length && same_final
    if comparable
        comparable &= all(
            i -> check(within(ref.t[i], other.t[i]; GRID...)), eachindex(ref.t)
        )
    end
    return comparable
end

"""State, control and variable of `other` against `ref` (grids must be comparable)."""
function compare_trajectories(ref::Data, other::Data; check_control::Bool=true)
    t = ref.t
    for i in eachindex(ref.xnames)
        check(within_L2(t, [s[i] for s in ref.x], [s[i] for s in other.x]; STATE...))
    end
    if check_control
        for i in eachindex(ref.unames)
            check(within_L2(t, [s[i] for s in ref.u], [s[i] for s in other.u]; CONTROL...))
        end
    end
    for i in eachindex(ref.vnames)
        check(within(ref.v[i], other.v[i]; VARIABLE...))
    end
    return nothing
end

function compare_objective(ref::Data, other::Data)
    return check(within(ref.objective, other.objective; OBJECTIVE...))
end

end # module Comparison
