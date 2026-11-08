# Comparison of the modelings of a problem (JuMP, OptimalControl, OptimalControlScalarized):
# do they describe the same problem? Used by `suite/comparison/`, at the initial guess
# (zero iterations): same structure, same grid, same trajectories, same objective.
#
# The solutions are deliberately not compared: after a solve, two modelings are two
# optimization paths, and a problem with several local solutions (`glider`, `cart_pendulum`)
# ends on one or the other depending on the platform. Model equivalence is the NLP
# equivalence of #284; convergence is tested per modeling.
# Numbers only: no plot. The visual comparison is left to the cron report (#310).
isdefined(Main, :SolveJuMP) || include(joinpath(@__DIR__, "solve_jump.jl"))
isdefined(Main, :SolveOptimalControl) ||
    include(joinpath(@__DIR__, "solve_optimalcontrol.jl"))

module Comparison

using Test: Test
using JuMP: JuMP
using OptimalControl: OptimalControl # loads the OptimalControlModels extension
using OptimalControlProblems: OptimalControlProblems

const OCP = OptimalControlProblems

# Tolerances (relative, absolute), as in the former `comparison`.
const GRID = (rtol=1e-2, atol=1e-6)
const OBJECTIVE = (rtol=1e-4, atol=1e-6)
const STATE = (rtol=1e-1, atol=1e-6)
const CONTROL = (rtol=1e-1, atol=1e-6)
const VARIABLE = (rtol=1e-1, atol=1e-6)

"""Every modeling is compared with the reference one."""
const REFERENCE = OCP.JuMPBackend()
const OTHERS = (OCP.OptimalControlBackend(), OCP.OptimalControlScalarizedBackend())

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

"""Solve `problem` with JuMP (solver of the first JuMP configuration)."""
function solve(::OCP.JuMPBackend, problem::Symbol; grid_size::Int, max_iter::Int)
    config = first(Main.TestConfigs.CONFIGS.JuMP)
    model = getfield(OCP, problem)(OCP.JuMPBackend(); grid_size=grid_size)
    Main.SolveJuMP.solve!(model; solver=config.solver, max_iter=max_iter)
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

"""
Solve `problem` with an OptimalControl modeling (first configuration of the modeling); the
size of the NLP is read from `discretize` + `nlp_model`, built as `solve` builds it.
"""
function solve(
    backend::OCP.AbstractModelBackend, problem::Symbol; grid_size::Int, max_iter::Int
)
    config = first(getfield(Main.TestConfigs.CONFIGS, Main.TestStatus.modeling(backend)))
    pb = getfield(OCP, problem)(backend)
    S = Main.SolveOptimalControl
    sol = S.solve(pb, config; grid_size=grid_size, max_iter=max_iter)
    return from_solution(sol, OCP.model(pb), S.nlp(pb, config; grid_size=grid_size))
end

"""
    for_each_comparison(f, test; grid_size, max_iter)

For every other modeling and every problem available in it and in the reference one,
solve both and call `f(problem, reference_data, other_data)`. A comparison is `:valid`
(a failure fails the run) only when the problem is `:valid` in both modelings.
"""
function for_each_comparison(f, test::AbstractString; grid_size, max_iter::Int)
    for other in OTHERS
        name = "$(Main.TestStatus.modeling(REFERENCE)) vs $(Main.TestStatus.modeling(other))"
        Test.@testset "$name" begin
            Main.TestStatus.for_each_problem(test, REFERENCE, other) do problem
                N = grid_size isa Function ? grid_size(problem) : grid_size
                ref = solve(REFERENCE, problem; grid_size=N, max_iter=max_iter)
                cmp = solve(other, problem; grid_size=N, max_iter=max_iter)
                f(problem, ref, cmp)
            end
        end
    end
    return nothing
end

# A comparison is a test: a failure fails the run for a `:valid` problem, and is recorded in
# the report for a `:draft` one (see `Main.TestStatus`). Returns the outcome.
check(ok::Bool) = (Test.@test ok; ok)

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
function compare_trajectories(ref::Data, other::Data)
    t = ref.t
    for i in eachindex(ref.xnames)
        check(within_L2(t, [s[i] for s in ref.x], [s[i] for s in other.x]; STATE...))
    end
    for i in eachindex(ref.unames)
        check(within_L2(t, [s[i] for s in ref.u], [s[i] for s in other.u]; CONTROL...))
    end
    for i in eachindex(ref.vnames)
        check(within(ref.v[i], other.v[i]; VARIABLE...))
    end
    return nothing
end

"""
Objectives agree. Written as one comparison so that a failure shows both values; the
iterations are logged then (a gap with many iterations means a solver that did not
converge or another local minimum).
"""
function compare_objective(ref::Data, other::Data)
    a, b = ref.objective, other.objective
    bound = max(0.5 * (abs(a) + abs(b)) * OBJECTIVE.rtol, OBJECTIVE.atol)
    ok = abs(a - b) < bound
    ok ||
        @warn "Objectives differ" reference = a other = b gap = abs(a - b) bound iterations = (
            ref.iterations, other.iterations
        )
    Test.@test abs(a - b) < bound
    return ok
end

end # module Comparison
