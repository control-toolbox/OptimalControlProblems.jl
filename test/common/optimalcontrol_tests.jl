# The tests of the OptimalControl modelings (OptimalControl and OptimalControlScalarized), written
# once and parameterized by the backend (#441): the API of a problem, the smoke solves, the
# convergence report. A test file of a modeling is a wrapper that gives its backend (TST1).
# The configurations come from `Main.TestConfigs.CONFIGS.<modeling>`, the solves from
# `Main.SolveOptimalControl`; the names of the test sets and of the lines of the report are
# those of the modeling, as before the tests were shared.
isdefined(Main, :SolveOptimalControl) ||
    include(joinpath(@__DIR__, "solve_optimalcontrol.jl"))

module OptimalControlTests

using Test: Test
using OptimalControl: OptimalControl # loads the OptimalControlModels extension
using OptimalControlProblems: OptimalControlProblems

const OCP = OptimalControlProblems
const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

"""Name of the modeling of `backend` (`:OptimalControl`, `:OptimalControlScalarized`)."""
modeling(backend) = Main.TestStatus.modeling(backend)

"""Configurations to solve with for `backend`, see `Main.TestConfigs.CONFIGS`."""
configs(backend) = getfield(Main.TestConfigs.CONFIGS, modeling(backend))

"""
`params` with its last nonzero numerical value changed by a relative 1e-6, small enough to
keep every bound of the problem ordered; `nothing` if there is no such value.
"""
function changed(params::NamedTuple)
    k = findlast(v -> v isa AbstractFloat || (v isa Integer && v != 0), values(params))
    isnothing(k) && return nothing
    key = keys(params)[k]
    return merge(params, NamedTuple{(key,)}((params[key] * (1 + 1e-6),)))
end

"""The API of a problem of the modeling of `backend`: the `Problem`, its metadata, its display."""
function test_problem(backend)
    name = modeling(backend)
    Test.@testset "$name Problem API" verbose = VERBOSE showtiming = SHOWTIMING begin
        Main.TestStatus.for_each_problem("problem", backend) do problem
            pb = getfield(OCP, problem)(backend)
            Test.@test pb isa OCP.Problem
            Test.@test OCP.model(pb) isa OptimalControl.Model
            Test.@test OCP.initial_guess(pb) isa NamedTuple
            Test.@test issubset(keys(OCP.initial_guess(pb)), (:state, :control, :variable))
            data = OCP.metadata(pb)
            Test.@test data[:parameters] == OCP.parameters_data(problem)
            Test.@test data[:status] == OCP.status(problem)
            Test.@test data[:grid_size] == OCP.grid_size_data(problem)

            # The parameters a problem is built with are recorded in its metadata, and the
            # metadata of the collection are left untouched.
            params = OCP.parameters_data(problem)
            new = isnothing(params) ? nothing : changed(params)
            if !isnothing(new)
                Test.@test new != params
                Test.@test OCP.metadata(getfield(OCP, problem)(backend; parameters=new))[:parameters] ==
                    new
                Test.@test OCP.parameters_data(problem) == params
            end

            # The initial guess is the one of the metadata (#377), unless another one is given.
            Test.@test OCP.initial_guess(pb) == OCP.initial_guess_data(problem)
            other = (state=OCP.initial_guess_data(problem).state, control=0)
            Test.@test OCP.initial_guess(getfield(OCP, problem)(backend; init=other)) === other

            # Shown with its name, modeling, status and dimensions.
            shown = sprint(show, MIME("text/plain"), pb)
            Test.@test occursin(string(problem), shown)
            Test.@test occursin("$name modeling", shown)
            Test.@test occursin(string(OCP.status(problem)[name]), shown)
            Test.@test occursin(
                "state dimension:    $(OCP.state_dimension(OCP.model(pb)))", shown
            )
            Test.@test sprint(show, pb) == "Problem(:$(problem))"
        end
    end
end

"""
Every problem builds, evaluates and runs a few iterations in each configuration; the
convergence is reported by the convergence tests (`convergence` argument, cron), #360.
"""
function test_smoke(backend)
    S = Main.SolveOptimalControl
    name = modeling(backend)
    Test.@testset "$name smoke (small grid, few iterations)" verbose = VERBOSE showtiming =
        SHOWTIMING begin
        for config in configs(backend)
            Test.@testset "$(Main.TestConfigs.label(config))" begin
                test = "test_smoke $(Main.TestConfigs.label(config))"
                Main.TestStatus.for_each_problem(test, backend) do problem
                    pb = getfield(OCP, problem)(backend)
                    o = Main.TestConfigs
                    sol = S.solve(pb, config; grid_size=o.SMOKE_GRID, max_iter=o.SMOKE_MAX_ITER)
                    Test.@test isfinite(OptimalControl.objective(sol))
                end
            end
        end
    end
end

"""
Every problem solved on its default grid in each configuration, and reported, not tested
(#360); an exception still fails.
"""
function test_convergence(backend)
    S = Main.SolveOptimalControl
    name = modeling(backend)
    Test.@testset "$name convergence report (default grid)" verbose = VERBOSE showtiming =
        SHOWTIMING begin
        for config in configs(backend)
            Test.@testset "$(Main.TestConfigs.label(config))" begin
                test = "test_convergence $(Main.TestConfigs.label(config))"
                Main.TestStatus.for_each_problem(test, backend) do problem
                    pb = getfield(OCP, problem)(backend)
                    N = OCP.grid_size_data(problem)
                    sol = S.solve(pb, config; grid_size=N)
                    time = @elapsed sol = S.solve(pb, config; grid_size=N) # second, compiled
                    # convergence is reported, not tested (#360); an exception still fails
                    Main.TestStatus.result!(
                        test, problem, backend, Main.TestConfigs.label(config);
                        S.settings(config)...,
                        grid_size=N, scheme=Main.TestConfigs.SCHEME, S.outcome(sol)...,
                        time=time, trajectory=Main.TestStatus.trajectory(sol),
                    )
                    Test.@test true
                end
            end
        end
    end
end

end # module OptimalControlTests
