module TestFeaturesComparison

using Test: Test
using JuMP: JuMP # loads the JuMPModels extension
using OptimalControl: OptimalControl # loads the OptimalControlModels extension
using OptimalControlProblems: OptimalControlProblems

const OCP = OptimalControlProblems
const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

# Model equivalence without a solve (#360): the vector and the scalarized OptimalControl
# models of a problem have the same features (#376): dimensions, cost, times, and the same
# number of constraints of each kind; and the three modelings have the same components and
# dimensions of the state, the control and the variable (#441). Where they are read: the
# JuMP model on a small grid, the continuous model of OptimalControl; nothing is solved.
function test_features()
    jump, vector, scalarized = OCP.JuMPBackend(), OCP.OptimalControlBackend(), OCP.OptimalControlScalarizedBackend()
    Test.@testset "features: OptimalControl vs OptimalControlScalarized" verbose = VERBOSE showtiming =
        SHOWTIMING begin
        Main.TestStatus.for_each_problem("test_features", vector, scalarized) do problem
            build = getfield(OCP, problem)
            Test.@test OCP.features(build(scalarized)) == OCP.features(build(vector))
        end
    end
    Test.@testset "components: JuMP vs OptimalControl" verbose = VERBOSE showtiming =
        SHOWTIMING begin
        for other in (vector, scalarized)
            Main.TestStatus.for_each_problem("test_components", jump, other) do problem
                build = getfield(OCP, problem)
                model = build(jump; grid_size=2)
                ocp = OCP.model(build(other))
                for f in (
                    OCP.state_components,
                    OCP.control_components,
                    OCP.variable_components,
                    OCP.state_dimension,
                    OCP.control_dimension,
                    OCP.variable_dimension,
                )
                    Test.@test f(model) == f(ocp)
                end
            end
        end
    end
end

end # module TestFeaturesComparison

test_features() = TestFeaturesComparison.test_features()
