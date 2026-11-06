module TestFeaturesComparison

using Test: Test
using OptimalControl: OptimalControl # loads the OptimalControlModels extension
using OptimalControlProblems: OptimalControlProblems

const OCP = OptimalControlProblems
const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

# Model equivalence without a solve (#360): the vector and the scalarized OptimalControl
# models of a problem have the same features (#376): dimensions, cost, times, and the same
# number of constraints of each kind.
function test_features()
    vector, scalarized = OCP.OptimalControlBackend(), OCP.OptimalControlScalarizedBackend()
    Test.@testset "features: OptimalControl vs OptimalControlScalarized" verbose = VERBOSE showtiming =
        SHOWTIMING begin
        Main.TestStatus.for_each_problem("test_features", vector, scalarized) do problem
            build = getfield(OCP, problem)
            Test.@test OCP.features(build(scalarized)) == OCP.features(build(vector))
        end
    end
end

end # module TestFeaturesComparison

test_features() = TestFeaturesComparison.test_features()
