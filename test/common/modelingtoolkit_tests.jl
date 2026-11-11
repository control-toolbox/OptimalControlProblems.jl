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
The objective of the analytic solution of `problem`, the reference of the gap in the report:
a shooting solve has no reference objective of its own discretization; `nothing` without one.
"""
function analytic_objective(problem, grid_size, scheme)
    ref = OCP.analytic_reference(problem)
    return isnothing(ref) ? nothing : ref.objective
end

"""What the shared tests need of the ModelingToolkit solves (see `common/modeling_tests.jl`)."""
function adapter()
    S = Main.SolveModelingToolkit
    return (
        solve=(pb, config; kwargs...) -> S.solve(pb, config; kwargs...),
        objective=S.objective,
        settings=S.settings,
        scheme=S.scheme,
        outcome=S.outcome,
        trajectory=S.trajectory,
        reference=analytic_objective,
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
test_problem(backend) =
    Main.ModelingTests.test_problem(backend; model_type=MTK.AbstractSystem, check=check)

"""Every problem builds and runs a few iterations; the convergence is in the cron (#360)."""
test_smoke(backend) = Main.ModelingTests.test_smoke(backend, adapter())

"""Every problem solved on its default grid, and reported, not tested (#360)."""
test_convergence(backend) = Main.ModelingTests.test_convergence(backend, adapter())

end # module ModelingToolkitTests
