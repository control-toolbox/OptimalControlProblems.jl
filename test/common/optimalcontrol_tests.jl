# The tests of the OptimalControl modelings (OptimalControl and OptimalControlScalarized), written
# once and parameterized by the backend (#441): the API of a problem, the smoke solves, the
# convergence report. A test file of a modeling is a wrapper that gives its backend (TST1).
# The tests themselves are shared with every modeling that returns a `Problem`
# (`common/modeling_tests.jl`, #439); this module gives them the OptimalControl solves
# (`Main.SolveOptimalControl`).
isdefined(Main, :SolveOptimalControl) ||
    include(joinpath(@__DIR__, "solve_optimalcontrol.jl"))
isdefined(Main, :ModelingTests) || include(joinpath(@__DIR__, "modeling_tests.jl"))

module OptimalControlTests

using OptimalControl: OptimalControl # loads the OptimalControlModels extension

"""What the shared tests need of the OptimalControl solves (see `common/modeling_tests.jl`)."""
function adapter()
    S = Main.SolveOptimalControl
    return (
        solve=(pb, config; kwargs...) -> S.solve(pb, config; kwargs...),
        objective=OptimalControl.objective,
        settings=S.settings,
        scheme=config -> Main.TestConfigs.SCHEME,
        outcome=S.outcome,
        trajectory=(sol, pb) -> Main.TestStatus.trajectory(sol),
        reference=(problem, grid_size, scheme) -> nothing,
    )
end

"""The API of a problem of the modeling of `backend`: the `Problem`, its metadata, its display."""
function test_problem(backend)
    return Main.ModelingTests.test_problem(backend; model_type=OptimalControl.Model)
end

"""
Every problem builds, evaluates and runs a few iterations in each configuration; the
convergence is reported by the convergence tests (`convergence` argument, cron), #360.
"""
test_smoke(backend) = Main.ModelingTests.test_smoke(backend, adapter())

"""
Every problem solved on its default grid in each configuration, and reported, not tested
(#360); an exception still fails.
"""
test_convergence(backend) = Main.ModelingTests.test_convergence(backend, adapter())

end # module OptimalControlTests
