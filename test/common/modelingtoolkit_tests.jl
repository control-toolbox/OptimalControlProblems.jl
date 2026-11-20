# The tests of the ModelingToolkit modeling: the tests every modeling that returns a `Problem`
# shares (`common/modeling_tests.jl`, #441), given the ModelingToolkit solves
# (`Main.SolveModelingToolkit`, Corleone and Ipopt), and the checks of a `System` (#439).
isdefined(Main, :SolveModelingToolkit) ||
    include(joinpath(@__DIR__, "solve_modelingtoolkit.jl"))
isdefined(Main, :ModelingTests) || include(joinpath(@__DIR__, "modeling_tests.jl"))

module ModelingToolkitTests

using Test: Test
using ModelingToolkit: ModelingToolkit as MTK # loads the ModelingToolkitModels extension
using OptimalControlProblems: OptimalControlProblems

const OCP = OptimalControlProblems

"""
The reference of the gap in the report: the reference objective of `problem` in
ModelingToolkit for this discretization (#435), else the objective of its analytic solution,
else `nothing`.
"""
function reference(problem, grid_size, scheme)
    backend = OCP.ModelingToolkitBackend()
    value = OCP.reference_objective(problem, backend; grid_size, scheme)
    isnothing(value) || return value
    ref = OCP.analytic_reference(problem)
    return isnothing(ref) ? nothing : ref.objective
end

"""
What the shared tests need of the ModelingToolkit solves (see `common/modeling_tests.jl`).
The warmup compiles on the smoke grid, a few iterations: a second solve on the default grid
costs as much as the first one (#435).
"""
function adapter()
    S = Main.SolveModelingToolkit
    return (
        solve=(pb, config; kwargs...) -> S.solve(pb, config; kwargs...),
        warmup=(pb, config) -> S.solve(
            pb,
            config;
            grid_size=Main.TestConfigs.SMOKE_GRID,
            max_iter=Main.TestConfigs.SMOKE_MAX_ITER,
        ),
        objective=S.objective,
        settings=S.settings,
        scheme=S.scheme,
        outcome=S.outcome,
        trajectory=S.trajectory,
        reference=reference,
    )
end

"""The checks of a `System`: its dimensions, one dynamics per state, a cost."""
function check(problem, pb)
    sys = OCP.model(pb)
    Test.@test OCP.state_dimension(sys) == length(OCP.state_components(sys)) > 0
    Test.@test OCP.control_dimension(sys) == length(OCP.control_components(sys)) > 0
    Test.@test OCP.variable_dimension(sys) == length(OCP.variable_components(sys))
    Test.@test length(MTK.equations(sys)) == OCP.state_dimension(sys)
    Test.@test !isempty(MTK.get_costs(sys))
end

"""The API of a problem of the modeling: the `Problem`, its metadata, its display."""
function test_problem(backend)
    return Main.ModelingTests.test_problem(
        backend; model_type=MTK.AbstractSystem, check=check
    )
end

"""Every problem builds and runs a few iterations; the convergence is in the cron (#360)."""
test_smoke(backend) = Main.ModelingTests.test_smoke(backend, adapter())

"""Every problem solved on its default grid, and reported, not tested (#360)."""
test_convergence(backend) = Main.ModelingTests.test_convergence(backend, adapter())

"""A draft warns when it is built, a valid problem does not."""
test_draft(backend) = Main.ModelingTests.test_draft(backend, adapter())

end # module ModelingToolkitTests
