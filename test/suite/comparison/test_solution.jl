isdefined(Main, :Comparison) ||
    include(joinpath(@__DIR__, "..", "..", "common", "comparison.jl"))

module TestSolutionComparison

using Test: Test
using CTBase: CTBase
using JuMP: JuMP
using OptimalControl: OptimalControl
using OptimalControlProblems: OptimalControlProblems

const OCP = OptimalControlProblems
const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

vec1(a::Number) = [Float64(a)]
vec1(a) = Float64.(a)

const SCHEMES = (:trapeze, :euler, :euler_implicit, :midpoint)

# The solution built from a JuMP model gives back what the JuMP accessors give. Zero
# iterations: the structure, not a solve (the solves are compared below, on `beam`).
function check_solution(problem::Symbol, scheme::Symbol)
    N = Main.TestConfigs.SMALL_GRID
    model = getfield(OCP, problem)(OCP.JuMPBackend(); grid_size=N, scheme=scheme)
    Main.SolveJuMP.solve!(model; max_iter=0)
    pb = getfield(OCP, problem)(OCP.OptimalControlBackend())
    sol = OCP.solution(model, pb)

    T = collect(Float64, OCP.time_grid(model))
    Test.@test OCP.model(sol) === OCP.model(pb)
    Test.@test collect(Float64, OCP.time_grid(sol)) == T
    Test.@test OCP.objective(sol) == OCP.objective(model)
    Test.@test OCP.iterations(sol) == OCP.iterations(model)
    Test.@test vec1(OCP.variable(sol)) == vec1(OCP.variable(model))
    Test.@test all(t -> vec1(OCP.state(sol)(t)) ≈ vec1(OCP.state(model)(t)), T)
    Test.@test all(t -> vec1(OCP.control(sol)(t)) ≈ vec1(OCP.control(model)(t)), T)
    Test.@test all(
        t -> vec1(OCP.costate(sol)(t)) ≈ vec1(OCP.costate(model)(t)), T[1:(end - 1)]
    )
    scheme === :trapeze && check_trajectory(model, sol)
    if scheme != :trapeze
        # one control per step, held on the step [t_i, t_{i+1}) and repeated at the last node
        name = first(OCP.control_components(model))
        U = JuMP.value.(collect(model[Symbol(name)]))
        Test.@test length(U) == N
        u(t) = first(vec1(OCP.control(sol)(t)))
        Test.@test [u((T[i] + T[i + 1]) / 2) for i in 1:N] == U
        Test.@test u(T[end]) == U[end]
    end
    return nothing
end

# The trajectory published for the documentation (#314) is what the accessors give, on at
# most `TRAJECTORY_POINTS` nodes, for a JuMP model as for an OptimalControl solution, to the
# 6 significant digits it is rounded to.
function check_trajectory(model, sol)
    rtol = 1e-5
    for x in (model, sol)
        tr = Main.TestStatus.trajectory(x)
        Test.@test !isnothing(tr)
        t = tr["time"]
        Test.@test length(t) <= Main.TestStatus.TRAJECTORY_POINTS
        Test.@test isapprox(t[1], first(OCP.time_grid(x)); rtol) &&
            isapprox(t[end], last(OCP.time_grid(x)); rtol)
        Test.@test sort(collect(keys(tr["state"]))) == sort(OCP.state_components(x))
        Test.@test sort(collect(keys(tr["control"]))) == sort(OCP.control_components(x))
        Test.@test tr["names"]["state"] == OCP.state_components(x)
        Test.@test tr["names"]["control"] == OCP.control_components(x)
        Test.@test length(tr["names"]["variable"]) == length(tr["variable"])
        for (i, name) in enumerate(OCP.state_components(x))
            Test.@test tr["state"][name] ≈ [vec1(OCP.state(x)(τ))[i] for τ in t] rtol = rtol
            Test.@test length(tr["costate"][name]) == length(t)
        end
        Test.@test tr["variable"] ≈ vec1(OCP.variable(x)) rtol = rtol
    end
    return nothing
end

function test_solution()
    Test.@testset "JuMP solution as a CTModels.Solution" verbose = VERBOSE showtiming =
        SHOWTIMING begin
        Main.TestStatus.for_each_problem(
            "test_solution", OCP.JuMPBackend(), OCP.OptimalControlBackend()
        ) do problem
            for scheme in SCHEMES
                Test.@testset "$scheme" check_solution(problem, scheme)
            end
        end

        Test.@testset "errors" begin
            pb = OCP.beam(OCP.OptimalControlBackend())
            # not solved
            unsolved = OCP.beam(OCP.JuMPBackend(); grid_size=Main.TestConfigs.SMALL_GRID)
            Test.@test_throws CTBase.Exceptions.PreconditionError OCP.solution(unsolved, pb)
            # a problem of another dimension
            model = OCP.beam(OCP.JuMPBackend(); grid_size=Main.TestConfigs.SMALL_GRID)
            Main.SolveJuMP.solve!(model; max_iter=0)
            other = OCP.glider(OCP.OptimalControlBackend())
            Test.@test_throws CTBase.Exceptions.IncorrectArgument OCP.solution(model, other)
        end

        Test.@testset "beam, solved, against OptimalControl, $scheme" for scheme in SCHEMES
            N = Main.TestConfigs.SMALL_GRID
            model = OCP.beam(OCP.JuMPBackend(); grid_size=N, scheme=scheme)
            Main.SolveJuMP.solve!(model)
            pb = OCP.beam(OCP.OptimalControlBackend())
            sol = OCP.solution(model, pb)
            Test.@test OptimalControl.successful(sol)

            # the same NLP (#369): solved by CTDirect with the same scheme
            ref = OptimalControl.solve(
                OCP.model(pb);
                init=OCP.initial_guess(pb),
                discretizer=OptimalControl.Collocation(; grid_size=N, scheme=scheme),
                modeler=OptimalControl.ADNLP(),
                solver=Main.SolveOptimalControl.solver(
                    :ipopt; max_iter=Main.TestConfigs.MAX_ITER
                ),
                display=false,
            )
            C = Main.Comparison
            Test.@test C.within(OCP.objective(sol), OCP.objective(ref); C.OBJECTIVE...)
            T = collect(Float64, OCP.time_grid(sol))
            for i in eachindex(first(vec1(OCP.state(ref)(0.0))))
                a = [vec1(OCP.state(sol)(t))[i] for t in T]
                b = [vec1(OCP.state(ref)(t))[i] for t in T]
                Test.@test C.within_L2(T, a, b; C.STATE...)
            end
            # costate on the intervals: same sign convention (#326, signs.md)
            for i in eachindex(first(vec1(OCP.costate(ref)(T[1]))))
                a = [vec1(OCP.costate(sol)(t))[i] for t in T[1:(end - 1)]]
                b = [vec1(OCP.costate(ref)(t))[i] for t in T[1:(end - 1)]]
                Test.@test C.within_L2(T[1:(end - 1)], a, b; C.COSTATE...)
            end
        end
    end
end

end # module TestSolutionComparison

test_solution() = TestSolutionComparison.test_solution()
