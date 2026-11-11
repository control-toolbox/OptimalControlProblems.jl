isdefined(Main, :Continuous) ||
    include(joinpath(@__DIR__, "..", "..", "common", "continuous.jl"))
isdefined(Main, :ContinuousModelingToolkit) ||
    include(joinpath(@__DIR__, "..", "..", "common", "continuous_modelingtoolkit.jl"))

module TestModelingsModelingToolkit

using Test: Test
using ModelingToolkit: ModelingToolkit # loads the ModelingToolkitModels extension
using OptimalControl: OptimalControl # loads the OptimalControlModels extension
using OptimalControlProblems: OptimalControlProblems

const OCP = OptimalControlProblems
const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

# Do the ModelingToolkit and the OptimalControl modelings of a problem describe the same
# problem? As `suite/comparison/test_modelings` for JuMP (#441), at the continuous level and
# without a solve: the `System` is read as the data of a JuMP problem
# (`common/continuous_modelingtoolkit.jl`) and compared by `Continuous.compare` on the times,
# the dynamics, the objective, the path and boundary constraints and the bounds, at points
# taken on the initial guess. Here and not in `comparison`: ModelingToolkit is opt-in (#439).
function test_modelings()
    mtk = OCP.ModelingToolkitBackend()
    Test.@testset "modelings: the same continuous problem" verbose = VERBOSE showtiming =
        SHOWTIMING begin
        for other in (OCP.OptimalControlBackend(), OCP.OptimalControlScalarizedBackend())
            Test.@testset "ModelingToolkit vs $(Main.TestStatus.modeling(other))" begin
                Main.TestStatus.for_each_problem("test_modelings", mtk, other) do problem
                    build = getfield(OCP, problem)
                    sys, pb = OCP.model(build(mtk)), build(other)
                    ocp = OCP.model(pb)
                    xs, us = OCP.state_components(ocp), OCP.control_components(ocp)
                    Test.@testset "components" begin
                        Test.@test sort(OCP.state_components(sys)) == sort(xs)
                        Test.@test sort(OCP.control_components(sys)) == sort(us)
                    end
                    data = Main.ContinuousModelingToolkit.problem_data(sys, xs, us)
                    checks = Main.Continuous.compare(data, ocp, OCP.initial_guess(pb))
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

end # module TestModelingsModelingToolkit

test_modelings() = TestModelingsModelingToolkit.test_modelings()
