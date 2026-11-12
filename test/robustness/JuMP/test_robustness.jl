isdefined(Main, :SolveJuMP) ||
    include(joinpath(@__DIR__, "..", "..", "common", "solve_jump.jl"))

module TestRobustnessJuMP

using Test: Test
using JuMP: JuMP
using OptimalControlProblems: OptimalControlProblems

const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

# Robustness of the initial guess (#378): every problem solved on its default grid under the
# solver options of `TestConfigs.ROBUSTNESS`. Reported, not tested (#360): which known
# solution each solve reached is in the report (`solution`); an exception still fails.
function test_robustness()
    C = Main.TestConfigs
    backend = OptimalControlProblems.JuMPBackend()
    Test.@testset "JuMP robustness report (default grid)" verbose = VERBOSE showtiming =
        SHOWTIMING begin
        for config in C.CONFIGS.JuMP, options in getfield(C.ROBUSTNESS, config.solver)
            label = C.label(merge(config, options))
            Test.@testset "$label" begin
                test = "test_robustness $label"
                Main.TestStatus.for_each_problem(test, backend) do problem
                    model = getfield(OptimalControlProblems, problem)(backend)
                    time = @elapsed Main.SolveJuMP.solve!(
                        model; solver=config.solver, options=options
                    )
                    Main.TestStatus.result!(
                        test,
                        problem,
                        backend,
                        label;
                        Main.SolveJuMP.settings(config; options=options)...,
                        grid_size=OptimalControlProblems.grid_size_data(problem),
                        scheme=C.SCHEME,
                        Main.SolveJuMP.outcome(model)...,
                        time=time,
                    )
                    Test.@test true
                end
            end
        end
    end
end

end # module TestRobustnessJuMP

test_robustness() = TestRobustnessJuMP.test_robustness()
