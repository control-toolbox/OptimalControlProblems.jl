# The tests of the JuMP modeling, written once in `common/modeling_tests.jl` (#449 T4) and
# given the JuMP solves (`Main.SolveJuMP`): the draft warning, the smoke solves, the
# convergence and robustness reports. A JuMP problem function returns the discretized model,
# built on its grid: the adapter builds it (`build`) before it is solved. A test file of the
# modeling is a wrapper (TST1).
isdefined(Main, :SolveJuMP) || include(joinpath(@__DIR__, "solve_jump.jl"))
isdefined(Main, :ModelingTests) || include(joinpath(@__DIR__, "modeling_tests.jl"))

module JuMPTests

using JuMP: JuMP
using OptimalControlProblems: OptimalControlProblems

const OCP = OptimalControlProblems

"""What the shared tests need of the JuMP solves (see `common/modeling_tests.jl`)."""
function adapter()
    S = Main.SolveJuMP
    return (
        build=(problem, grid_size) -> getfield(OCP, problem)(OCP.JuMPBackend(); grid_size),
        solve=(model, config; grid_size, kwargs...) ->
            S.solve!(model; solver=config.solver, kwargs...),
        objective=model -> JuMP.has_values(model) ? JuMP.objective_value(model) : NaN,
        settings=S.settings,
        scheme=config -> Main.TestConfigs.SCHEME,
        outcome=S.outcome,
        trajectory=(model, problem) -> Main.TestStatus.trajectory(model),
        reference=(problem, grid_size, scheme) -> nothing,
    )
end

test_draft() = Main.ModelingTests.test_draft(OCP.JuMPBackend(), adapter())
test_smoke() = Main.ModelingTests.test_smoke(OCP.JuMPBackend(), adapter())
test_convergence() = Main.ModelingTests.test_convergence(OCP.JuMPBackend(), adapter())
test_robustness() = Main.ModelingTests.test_robustness(OCP.JuMPBackend(), adapter())

end # module JuMPTests
