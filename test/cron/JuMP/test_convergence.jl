isdefined(Main, :SolveJuMP) || include(joinpath(@__DIR__, "..", "..", "common", "solve_jump.jl"))

module TestConvergenceJuMP

using Test: Test
using JuMP: JuMP
using OptimalControlProblems: OptimalControlProblems

const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

function test_convergence()
    Test.@testset "JuMP convergence report (default grid)" verbose = VERBOSE showtiming = SHOWTIMING begin
        for config in Main.TestConfigs.CONFIGS.JuMP
            Test.@testset "$(Main.TestConfigs.label(config))" begin
                test = "test_convergence $(Main.TestConfigs.label(config))"
                Main.TestStatus.for_each_problem(test, OptimalControlProblems.JuMPBackend()) do problem
                    build = getfield(OptimalControlProblems, problem)
                    model = build(OptimalControlProblems.JuMPBackend())
                    Main.SolveJuMP.solve!(model; solver=config.solver)
                    time = @elapsed Main.SolveJuMP.solve!(model; solver=config.solver) # warm
                    # convergence is reported, not tested (#360); an exception still fails
                    Main.TestStatus.result!(
                        test, problem, OptimalControlProblems.JuMPBackend(),
                        Main.TestConfigs.label(config); Main.SolveJuMP.outcome(model)...,
                        time=time,
                    )
                    Test.@test true
                end
            end
        end
    end
end

end # module TestConvergenceJuMP

test_convergence() = TestConvergenceJuMP.test_convergence()
