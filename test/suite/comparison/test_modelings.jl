isdefined(Main, :Continuous) ||
    include(joinpath(@__DIR__, "..", "..", "common", "continuous.jl"))

module TestModelingsComparison

using Test: Test
using JuMP: JuMP # loads the JuMPModels extension
using OptimalControl: OptimalControl # loads the OptimalControlModels extension
using OptimalControlProblems: OptimalControlProblems

const OCP = OptimalControlProblems
const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

# Do the modelings of a problem describe the same problem? The JuMP modeling against the
# OptimalControl one and the scalarized one, at the continuous level and without discretizing
# (#441): times, dynamics, objective, path and boundary constraints, bounds, evaluated at the
# same points taken on the initial guess. Every problem: nothing is compiled for a
# discretization or an NLP. The discretization is compared with CTDirect's by `test_equivalence`,
# on a few problems.
function test_modelings()
    jump = OCP.JuMPBackend()
    Test.@testset "modelings: the same continuous problem" verbose = VERBOSE showtiming =
        SHOWTIMING begin
        for other in (OCP.OptimalControlBackend(), OCP.OptimalControlScalarizedBackend())
            Test.@testset "JuMP vs $(Main.TestStatus.modeling(other))" begin
                Main.TestStatus.for_each_problem("test_modelings", jump, other) do problem
                    build = getfield(OCP, problem)
                    data = build(jump; grid_size=2)[:problem_data]
                    pb = build(other)
                    checks = Main.Continuous.compare(data, OCP.model(pb), OCP.initial_guess(pb))
                    for (name, ok) in sort!(collect(checks); by=first)
                        Test.@testset "$name" begin
                            Test.@test ok
                        end
                    end
                end
            end
        end
    end
end

end # module TestModelingsComparison

test_modelings() = TestModelingsComparison.test_modelings()
