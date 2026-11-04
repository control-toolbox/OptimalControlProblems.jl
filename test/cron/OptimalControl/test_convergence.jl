isdefined(Main, :SolveIpopt) || include(joinpath(@__DIR__, "..", "..", "common", "solve_ipopt.jl"))

module TestConvergenceOptimalControl

using Test: Test
using CTDirect: CTDirect
using OptimalControl: OptimalControl # loads the OptimalControlModels extension
using OptimalControlProblems: OptimalControlProblems

const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

# Solver modules of test/common/, by the `solver` field of a configuration.
solver_module(config) = getfield(Main, Dict(:ipopt => :SolveIpopt, :madnlp => :SolveMadNLP)[config.solver])

function test_convergence()
    Test.@testset "OptimalControl convergence (default grid)" verbose = VERBOSE showtiming = SHOWTIMING begin
        for config in Main.TestConfigs.CONFIGS.OptimalControl
            Test.@testset "$(Main.TestConfigs.label(config))" begin
                solver = solver_module(config)
                test = "test_convergence $(Main.TestConfigs.label(config))"
                Main.TestStatus.for_each_problem(test, OptimalControlProblems.OptimalControlBackend()) do problem
                    build = getfield(OptimalControlProblems, problem)
                    docp = build(OptimalControlProblems.OptimalControlBackend(), config.description...)
                    nlp = CTDirect.nlp_model(docp)
                    stats = solver.solve(nlp)
                    stats = solver.solve(nlp) # second solve
                    Test.@test solver.succeeded(stats)
                end
            end
        end
    end
end

end # module TestConvergenceOptimalControl

test_convergence() = TestConvergenceOptimalControl.test_convergence()
