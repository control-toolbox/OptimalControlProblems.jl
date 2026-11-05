isdefined(Main, :SolveJuMP) || include(joinpath(@__DIR__, "..", "..", "common", "solve_jump.jl"))

module TestSolveJuMP

using Test: Test
using JuMP: JuMP
using OptimalControlProblems: OptimalControlProblems

const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

function test_solve()
    Test.@testset "JuMP solve (small grid)" verbose = VERBOSE showtiming = SHOWTIMING begin
        for config in Main.TestConfigs.CONFIGS.JuMP
            Test.@testset "$(Main.TestConfigs.label(config))" begin
                test = "test_solve $(Main.TestConfigs.label(config))"
                Main.TestStatus.for_each_problem(test, OptimalControlProblems.JuMPBackend()) do problem
                    build = getfield(OptimalControlProblems, problem)
                    model = build(OptimalControlProblems.JuMPBackend(); grid_size=Main.TestConfigs.SMALL_GRID)
                    Main.SolveJuMP.solve!(model; solver=config.solver)
                    Test.@test Main.SolveJuMP.succeeded(model)
                end
            end
        end
    end
end

end # module TestSolveJuMP

test_solve() = TestSolveJuMP.test_solve()
